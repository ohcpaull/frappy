# *****************************************************************************
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Module authors:
#   Oliver Paull <oliverp@ansto.gov.au>
# *****************************************************************************
"""Tests for the Caylar driver's reply handling and for the simulator.

Two levels, neither of which needs hardware or a Frappy server:

* ``CaylarIO``'s reply checking is exercised against a stub that returns
  canned replies - the reply strings are taken verbatim from the examples in
  the Ethernet Interface Manual, so these tests fail if the parsing drifts
  away from what the supply actually says.
* the simulator is driven over a real socket, which keeps it honest as a
  stand-in for the supply.
"""

import re
import socket
import threading
import time
from types import SimpleNamespace

import pytest

from frappy.core import BUSY, ERROR, IDLE, WARN
from frappy.errors import BadValueError, CommunicationFailedError, \
    ImpossibleError, ProgrammingError, RangeError
from frappy_acns.caylar import POWER_CONFIG, POWER_LIMITS, RAMP_MODE, \
    REGULATION_MODE, SETPOINT_SOURCE, CaylarIO, CaylarModule, CaylarRegulation
from frappy_acns.caylar_sim import CaylarServer


class StubIO:
    """CaylarIO's reply handling, without a Frappy server underneath it.

    The three methods under test only use ``self.communicate``, so borrowing
    them onto a stub exercises the real code rather than a copy of it.
    """
    execute = CaylarIO.execute
    query = CaylarIO.query
    echo_matches = staticmethod(CaylarIO.echo_matches)
    _raise_reply_error = CaylarIO._raise_reply_error

    def __init__(self, reply):
        self.reply = reply
        self.sent = None

    def communicate(self, request):
        self.sent = request
        return self.reply


# ------------------------------------------------------- reply parsing: GET

def test_query_strips_the_get_prefix_from_the_echo():
    io = StubIO('CURRENT= +123.456789 A')
    assert io.query('CURRENT') == ['+123.456789', 'A']
    assert io.sent == 'GET_CURRENT'


def test_query_returns_every_field_after_the_echo():
    # GET_CURRENT_SETPOINT_INTERVAL answers with min, max and a unit
    io = StubIO('CURRENT_SETPOINT_INTERVAL= -30.000 30.000 A')
    assert io.query('CURRENT_SETPOINT_INTERVAL') == ['-30.000', '30.000', 'A']


def test_query_accepts_a_per_configuration_echo_that_keeps_the_suffix():
    io = StubIO('USER_MAX_CURRENT_80V140A= 100.0000 A')
    assert io.query('USER_MAX_CURRENT_80V140A')[0] == '100.0000'


@pytest.mark.parametrize('item, reply', [
    # the supply drops the configuration suffix from these echoes - both the
    # manual and real hardware do it, so the driver must not insist on it
    ('DIGITAL_CURRENT_RAMP_SPEED_90V30A', 'DIGITAL_CURRENT_RAMP_SPEED= 02.50 A/Sec'),
    ('DIGITAL_CURRENT_RAMP_SPEED_80V140A', 'DIGITAL_CURRENT_RAMP_SPEED= 05.50 A/Sec'),
    ('ANALOG_CURRENT_RAMP_SPEED_90V30A', 'ANALOG_CURRENT_RAMP_SPEED= 10.00 A/Sec'),
])
def test_query_accepts_a_per_configuration_echo_that_drops_the_suffix(item, reply):
    assert StubIO(reply).query(item)[0] == reply.split()[1]


def test_execute_accepts_a_per_configuration_echo_that_drops_the_suffix():
    io = StubIO('SET_DIGITAL_CURRENT_RAMP_SPEED_OK 02.50 A/Sec')
    assert io.execute('SET_DIGITAL_CURRENT_RAMP_SPEED_90V30A', 2.5) == ['02.50', 'A/Sec']
    assert io.sent == 'SET_DIGITAL_CURRENT_RAMP_SPEED_90V30A 2.5'


def test_execute_accepts_a_per_configuration_echo_that_keeps_the_suffix():
    io = StubIO('SET_USER_MAX_CURRENT_90V30A_OK 002.0000 A')
    assert io.execute('SET_USER_MAX_CURRENT_90V30A', 2) == ['002.0000', 'A']


@pytest.mark.parametrize('echo, name', [
    # exactly as sent
    ('CURRENT', 'CURRENT'),
    ('USER_MAX_CURRENT_90V30A', 'USER_MAX_CURRENT_90V30A'),
    # suffix dropped from the echo
    ('DIGITAL_CURRENT_RAMP_SPEED', 'DIGITAL_CURRENT_RAMP_SPEED_90V30A'),
    ('ANALOG_CURRENT_RAMP_SPEED', 'ANALOG_CURRENT_RAMP_SPEED_80V140A'),
    # suffix respelled with an underscore the command did not have
    ('FIELD_RAMP_SPEED_90V_30A', 'FIELD_RAMP_SPEED_90V30A'),
    ('FIELD_RAMP_SPEED_80V_140A', 'FIELD_RAMP_SPEED_80V140A'),
    ('USER_MAX_CURRENT_90V_30A', 'USER_MAX_CURRENT_90V30A'),
])
def test_echo_matches_absorbs_the_supplys_spelling(echo, name):
    assert CaylarIO.echo_matches(echo, name)


@pytest.mark.parametrize('echo, name', [
    ('USER_MAX_FIELD', 'USER_MAX_CURRENT_90V30A'),
    ('USER_MAX_FIELD_90V30A', 'USER_MAX_CURRENT_90V30A'),
    ('DIGITAL_CURRENT_RAMP_SPEED', 'ANALOG_CURRENT_RAMP_SPEED_90V30A'),
    ('CURRENT', 'VOLTAGE'),
    ('CURRENT_SETPOINT', 'CURRENT'),
])
def test_echo_matches_still_rejects_a_different_command(echo, name):
    """being relaxed about the suffix must not mean being relaxed about which
    command answered - that is the whole point of checking the echo"""
    assert not CaylarIO.echo_matches(echo, name)


@pytest.mark.parametrize('reply, item', [
    ('FIELD_RAMP_SPEED_90V_30A= 0250.00 G/Sec', 'FIELD_RAMP_SPEED_90V30A'),
    ('USER_MAX_CURRENT_90V_30A= 030.0000 A', 'USER_MAX_CURRENT_90V30A'),
])
def test_query_accepts_a_respelled_suffix(reply, item):
    """'GET_FIELD_RAMP_SPEED_90V30A' is answered with 'FIELD_RAMP_SPEED_90V_30A='"""
    assert StubIO(reply).query(item)[0] == reply.split()[1]


def test_execute_accepts_a_respelled_suffix():
    io = StubIO('SET_FIELD_RAMP_SPEED_90V_30A_OK 0250.00 G/Sec')
    assert io.execute('SET_FIELD_RAMP_SPEED_90V30A', 250) == ['0250.00', 'G/Sec']


def test_a_reply_for_a_different_command_is_still_rejected():
    with pytest.raises(CommunicationFailedError):
        StubIO('USER_MAX_FIELD= 20000.0 G').query('USER_MAX_CURRENT_90V30A')
    with pytest.raises(CommunicationFailedError):
        StubIO('DIGITAL_CURRENT_RAMP_SPEED= 02.50 A/Sec').query('ANALOG_CURRENT_RAMP_SPEED_90V30A')


def test_low_water_flow_reading_is_not_a_number():
    """the supply answers '<2.5' below the sensor threshold, not a float

    Worth pinning: a plain float() on this raises, and the resulting
    ValueError would be reported as a driver crash rather than as the low
    flow it actually is.
    """
    io = StubIO('WATER_FLOW= <2.5 L/Min')
    reading = io.query('WATER_FLOW')[0]
    assert reading == '<2.5'
    assert float(reading.lstrip('<')) == 2.5


# ------------------------------------------------------- reply parsing: SET

def test_execute_checks_the_ok_echo_and_returns_the_readback():
    io = StubIO('SET_CURRENT_OK +10.250000 A')
    assert io.execute('SET_CURRENT', 10.25) == ['+10.250000', 'A']
    assert io.sent == 'SET_CURRENT 10.25'


def test_execute_joins_multiple_arguments_with_single_spaces():
    io = StubIO('SET_FIELD_REGUL_KP_OK 0.025000')
    io.execute('SET_FIELD_REGUL_KP', 0, 0.025)
    assert io.sent == 'SET_FIELD_REGUL_KP 0 0.025'


def test_execute_accepts_a_bare_ok_with_no_readback():
    io = StubIO('SET_POWER_ON_OK')
    assert io.execute('SET_POWER_ON') == []


# -------------------------------------------------------------- error paths

@pytest.mark.parametrize('reply, expected', [
    ('SET_CURRENT_ERROR OVERRANGE', RangeError),
    ('SET_CURRENT_ERROR BAD_ARG', BadValueError),
    ('SET_CURRENT_ERROR FIELD_REG', ImpossibleError),
    ('SET_CURRENT_ERROR MAINTENANCE_ON', ImpossibleError),
    ('SET_POWER_ON_ERROR DEFAULT_ON', ImpossibleError),
    ('SET_REGUL_FIELD_ERROR EXTERNAL_SETPOINT_SELEC', ImpossibleError),
    ('SET_USER_MAX_CURRENT_ERROR MAX_CURRENT_OVERRANGE', RangeError),
    ('SET_FIELD_REGUL_KP_ERROR UNKNOWN_PWR_CONFIG', ProgrammingError),
])
def test_error_replies_become_the_matching_secop_error(reply, expected):
    with pytest.raises(expected):
        StubIO(reply).execute('SET_CURRENT', 1)


def test_the_manuals_odd_ok_error_reply_is_still_an_error():
    """SET_USER_MAX_CURRENT documents 'SET_USER_MAX_CURRENT_OK_ERROR BAD_ARG'

    Almost certainly a typo in the manual, but a reply ending in _ERROR must
    not be mistaken for a success whatever precedes it.
    """
    with pytest.raises(BadValueError):
        StubIO('SET_USER_MAX_CURRENT_OK_ERROR BAD_ARG').execute('SET_USER_MAX_CURRENT', 2)


def test_unknown_error_reason_still_raises():
    with pytest.raises(CommunicationFailedError):
        StubIO('SET_CURRENT_ERROR SOMETHING_NEW').execute('SET_CURRENT', 1)


def test_unrecognised_command_raises_programming_error():
    with pytest.raises(ProgrammingError):
        StubIO('WRONGCOMMAND').query('NOT_A_THING')


def test_reply_for_a_different_command_is_not_accepted():
    """a reply that does not echo our command means we are out of step"""
    with pytest.raises(CommunicationFailedError):
        StubIO('VOLTAGE= +12.345 V').query('CURRENT')


def test_empty_reply_raises():
    with pytest.raises(CommunicationFailedError):
        StubIO('').query('CURRENT')


# ----------------------------------------------------- per-configuration bits

@pytest.mark.parametrize('config', ['80V140A', '90V30A'])
def test_cfg_command_appends_the_power_configuration(config):
    module = CaylarModule.__new__(CaylarModule)
    module._power_config = config
    assert module.cfg_command('USER_MAX_CURRENT') == f'USER_MAX_CURRENT_{config}'


def test_every_selectable_configuration_has_limits():
    """POWER_LIMITS is indexed by POWER_CONFIG member name, so it must cover
    every member - a missing one is a KeyError at module startup, not a
    warning. Extra keys are allowed: they exist as command-name aliases.
    """
    for member in POWER_CONFIG.members:
        assert member.name in POWER_LIMITS, f'no limits for {member.name}'


def test_the_native_output_ranges_match_the_manual():
    assert POWER_LIMITS['80V140A']['max_current'] == 140.0
    assert POWER_LIMITS['80V140A']['max_voltage'] == 80.0
    assert POWER_LIMITS['90V30A']['max_current'] == 30.0
    assert POWER_LIMITS['90V30A']['max_voltage'] == 90.0


# ----------------------------------------------- the target follows the panel

class StubRegulation:
    """CaylarRegulation's status logic with the supply and hardware stubbed.

    Exists to pin one thing that is easy to break by rearranging read_status:
    the target must be refreshed from the hardware on *every* path through it,
    including the ones that return early. Somebody changing the setpoint at the
    front panel is most likely to be doing it exactly when the supply is off or
    faulted, which are the early-return paths.
    """
    read_status = CaylarRegulation.read_status
    is_ramping = CaylarRegulation.is_ramping
    Status = CaylarRegulation.Status
    regulation_mode = REGULATION_MODE.current

    def __init__(self, supply_status=(IDLE, ''), **supply):
        self.target_reads = 0
        self.hardware_setpoint = 42.0
        self.value = 42.0
        self.tolerance = 0.05
        self.supply = SimpleNamespace(
            status=supply_status,
            maintenance=supply.get('maintenance', False),
            setpoint_source=supply.get('setpoint_source', SETPOINT_SOURCE.internal),
            regulation_mode=supply.get('regulation_mode', REGULATION_MODE.current),
            ramp_mode=supply.get('ramp_mode', RAMP_MODE.analog),
        )

    def read_target(self):
        self.target_reads += 1
        return self.hardware_setpoint


@pytest.mark.parametrize('name, kwargs', [
    ('a latched fault', {'supply_status': (ERROR, 'fault: WATER_FLOW')}),
    ('the power off', {'supply_status': (StubRegulation.Status.DISABLED, 'power off')}),
    ('maintenance mode', {'supply_status': (WARN, 'maintenance'), 'maintenance': True}),
    ('an external setpoint', {'setpoint_source': SETPOINT_SOURCE.external}),
    ('the other regulation mode', {'regulation_mode': REGULATION_MODE.field}),
])
def test_target_is_refreshed_whatever_the_status(name, kwargs):
    module = StubRegulation(**kwargs)
    module.read_status()
    assert module.target_reads == 1, f'target not refreshed with {name}'


def test_target_is_refreshed_exactly_once_per_status_poll():
    """the ramping test reuses the reading rather than making a second query"""
    module = StubRegulation()
    module.read_status()
    assert module.target_reads == 1


def test_status_still_reports_what_the_supply_says():
    """refreshing the target must not swallow the supply's own status"""
    assert StubRegulation(supply_status=(ERROR, 'fault: X')).read_status() == (ERROR, 'fault: X')
    assert StubRegulation(setpoint_source=SETPOINT_SOURCE.external).read_status()[0] == WARN
    assert StubRegulation(regulation_mode=REGULATION_MODE.field).read_status()[0] == WARN


def test_busy_while_the_value_is_away_from_the_refreshed_target():
    module = StubRegulation()
    module.hardware_setpoint = 100.0   # as if typed at the front panel
    module.value = 42.0
    assert module.read_status() == (BUSY, 'ramping')


def test_idle_once_the_value_is_within_tolerance_of_the_target():
    module = StubRegulation()
    module.hardware_setpoint = 42.02
    module.value = 42.0                # inside the 0.05 tolerance
    assert module.read_status() == (IDLE, '')


# ------------------------------------------------------------ the simulator

@pytest.fixture(name='sim')
def sim_fixture(request):
    """a simulated supply on an ephemeral port, with a talk() helper"""
    config = getattr(request, 'param', '80V140A')
    server = CaylarServer(('127.0.0.1', 0), config)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    conn = socket.create_connection(server.server_address, timeout=5)
    stream = conn.makefile('rw', newline='\n')

    def talk(command):
        stream.write(command + '\n')
        stream.flush()
        return stream.readline().strip()

    talk.server = server
    yield talk

    conn.close()
    server.shutdown()
    server.server_close()


def test_sim_identifies_the_way_the_real_unit_does(sim):
    """bare serial number, no 'CAYLAR_' prefix

    The manual documents 'CAYLAR_<serial number>' but the ANSTO unit does not
    send the prefix, which is why CaylarIO.identification matches on the serial
    number instead. The simulator has to be wrong in the same way or it would
    not catch a regression in that regex.
    """
    reply = sim('*IDN?')
    assert reply.startswith('8220')
    # identification is a frappy Property, so the configured value lives on the
    # descriptor rather than on the class attribute
    _, regexp = CaylarIO.identification.value[0]
    assert re.match(regexp, reply), f'{reply!r} does not match {regexp!r}'


def test_sim_answers_a_get_with_the_manuals_echo_format(sim):
    assert sim('GET_CURRENT').startswith('CURRENT= ')
    assert sim('GET_VOLTAGE').startswith('VOLTAGE= ')
    assert sim('GET_POWER_CONFIGURATION') == 'POWER_CONFIGURATION= 0'


@pytest.mark.parametrize('sim', ['90V30A'], indirect=True)
def test_sim_reports_the_configuration_it_was_started_with(sim):
    assert sim('GET_POWER_CONFIGURATION') == 'POWER_CONFIGURATION= 1'


def test_sim_rejects_an_unknown_command(sim):
    assert sim('GET_THE_MOON') == 'WRONGCOMMAND'


def test_sim_refuses_a_field_setpoint_while_regulating_on_current(sim):
    assert sim('SET_REGUL_CURRENT') == 'SET_REGUL_CURRENT_OK'
    assert sim('SET_FIELD 100') == 'SET_FIELD_ERROR CURRENT_REG'


def test_sim_enforces_the_user_current_limit(sim):
    assert sim('SET_CURRENT 10.25') == 'SET_CURRENT_OK +10.250000 A'
    assert sim('SET_CURRENT 999') == 'SET_CURRENT_ERROR OVERRANGE'


def test_sim_keeps_ramp_speeds_per_power_configuration(sim):
    """the two configurations must not share a stored setting"""
    assert sim('SET_DIGITAL_CURRENT_RAMP_SPEED_80V140A 7.5').endswith('A/Sec')
    assert '07.5' in sim('GET_DIGITAL_CURRENT_RAMP_SPEED_80V140A')
    assert '07.5' not in sim('GET_DIGITAL_CURRENT_RAMP_SPEED_90V30A')


def test_sim_reproduces_the_inconsistent_suffix_echo(sim):
    """all three spellings the supply uses for the per-config suffix

    Pinned here because it is the simulator's job to be awkward in the same
    way the hardware is - if this ever gets "tidied up", the driver's handling
    of the odd spellings stops being tested.
    """
    # dropped
    assert sim('GET_DIGITAL_CURRENT_RAMP_SPEED_80V140A').startswith(
        'DIGITAL_CURRENT_RAMP_SPEED= ')
    assert sim('GET_ANALOG_CURRENT_RAMP_SPEED_80V140A').startswith(
        'ANALOG_CURRENT_RAMP_SPEED= ')
    # respelled with an extra underscore
    assert sim('GET_FIELD_RAMP_SPEED_80V140A').startswith(
        'FIELD_RAMP_SPEED_80V_140A= ')
    # kept exactly as sent
    assert sim('GET_USER_MAX_CURRENT_80V140A').startswith(
        'USER_MAX_CURRENT_80V140A= ')


def test_sim_blocks_every_set_while_in_maintenance(sim):
    assert sim('SET_MAINTENANCE_ON') == 'SET_MAINTENANCE_ON_OK'
    assert sim('GET_MAINTENANCE_STATE') == 'MAINTENANCE_STATE= 1'
    assert sim('SET_CURRENT 1') == 'SET_CURRENT_ERROR MAINTENANCE_ON'
    assert sim('SET_POWER_ON') == 'SET_POWER_ON_ERROR MAINTENANCE_ON'
    # reads still work: maintenance stops control, not monitoring
    assert sim('GET_CURRENT').startswith('CURRENT= ')


def test_sim_latches_a_user_fault_until_it_is_cleared(sim):
    assert sim('SET_POWER_ON') == 'SET_POWER_ON_OK'
    assert sim('SET_DEFAULT_ON') == 'SET_DEFAULT_ON_OK'
    assert sim('GET_DEFAULT_STATE') == 'DEFAULT_STATE= 1'
    assert sim('GET_POWER_STATE') == 'POWER_STATE= 0'
    # powering back on is refused while the fault is latched
    assert sim('SET_POWER_ON') == 'SET_POWER_ON_ERROR DEFAULT_ON'
    assert sim('SET_DEFAULT_OFF') == 'SET_DEFAULT_OFF_OK'
    assert sim('GET_DEFAULT_STATE') == 'DEFAULT_STATE= 1'  # still latched
    assert sim('CLEAR_DEFAULT') == 'CLEAR_DEFAULT_OK'
    assert sim('GET_DEFAULT_STATE') == 'DEFAULT_STATE= 0'
    assert sim('SET_POWER_ON') == 'SET_POWER_ON_OK'


def test_sim_ramps_the_output_towards_the_setpoint(sim):
    """the output must travel to the setpoint rather than jump to it

    The driver reports BUSY off the back of this, so a simulator that arrived
    instantly would make the ramping path untestable.
    """
    supply = sim.server.supply
    sim('SET_POWER_ON')
    sim('SET_REGUL_CURRENT')
    sim('SET_DIGITAL_CURRENT_RAMP_SPEED_80V140A 20')
    sim('SET_CURRENT 10')  # 10 A at 20 A/s: about half a second

    time.sleep(0.15)
    assert 0 < supply.current < 10

    deadline = time.time() + 10
    while supply.is_ramping() and time.time() < deadline:
        time.sleep(0.05)
    assert supply.current == pytest.approx(10, abs=0.01)


def test_sim_zeroes_the_output_when_the_power_is_off(sim):
    """no output without power - what the power_off guard relies on"""
    supply = sim.server.supply
    sim('SET_POWER_ON')
    sim('SET_REGUL_CURRENT')
    sim('SET_CURRENT 5')
    time.sleep(0.3)
    assert supply.current > 0
    sim('SET_POWER_OFF')
    time.sleep(0.3)
    assert supply.current == 0
