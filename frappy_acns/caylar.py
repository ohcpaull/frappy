#!/usr/bin/env python
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
"""Caylar MPU8220-072 / -073 electromagnet power supply.

Protocol reference: "Ethernet Interface Manual", AF202303-010, rev 4.3.

The supply is a bipolar current source for an electromagnet, with a Hall probe
for closed-loop field regulation.  It talks line-oriented ASCII over a plain TCP
socket on port 1234, so ``CaylarIO`` is a ``StringIO``.

Two things about this supply drive the whole shape of this driver:

1. **It has two power configurations**, +/-80 V +/-140 A and +/-90 V +/-30 A,
   selected by a *physical selector on the front panel*.  Nothing in the
   protocol can change it - ``GET_POWER_CONFIGURATION`` only reports it.  Many
   commands are per-configuration (``..._80V140A`` / ``..._90V30A``) and each
   configuration keeps its own stored ramp speeds and user limits.  So the
   configuration is declared in the cfg file - it decides the current limits
   this node enforces - and ``CaylarIO`` verifies it against the hardware on
   every connect, refusing to run if the selector disagrees.

2. **It regulates on either current or field, not both.**  A SECoP parameter has
   one fixed datatype, so a single module cannot swap its value between amps and
   gauss at runtime.  Instead there are two Drivables - :class:`CaylarCurrent`
   and :class:`CaylarField` - and writing a target on one switches the supply
   into that regulation mode.  :class:`CaylarSupply` carries everything that
   belongs to neither: power state, faults, temperatures, cooling water.

All three share one ``CaylarIO`` and therefore one TCP connection.  Because the
power configuration lives on the io module, the cfg file declares that module
explicitly rather than using the ``uri=`` shortcut - see ``cfg/caylar_cfg.py``.

A note on vocabulary: the protocol says "DEFAULT" where it means *fault*
("defaut" in the manufacturer's French).  ``GET_DEFAULT_STATE`` is a fault flag,
not a factory reset.  The parameters here are named ``fault`` / ``fault_name``
and the command ``clear_fault``, with the raw protocol names kept only in the
strings actually sent down the wire.

PID tuning of the field regulation (``SET_FIELD_REGUL_KP`` and friends) is
deliberately not exposed yet.
"""

import time

from frappy.core import BUSY, ERROR, IDLE, WARN, Attached, BoolType, Command, \
    Drivable, Enum, EnumType, FloatRange, HasIO, Parameter, Property, \
    Readable, StatusType, StringIO, StringType, nopoll
from frappy.errors import BadValueError, CommunicationFailedError, \
    ConfigError, ImpossibleError, ProgrammingError, RangeError

# The enums the protocol works in.  Defined at module level so that the code
# below can say REGULATION_MODE.field rather than digging the datatype out of a
# parameter every time it needs to compare one.
POWER_CONFIG = Enum('power_config', {'80V140A': 0, '90V30A': 1})
REGULATION_MODE = Enum('regulation_mode', {'current': 0, 'field': 1})
SETPOINT_SOURCE = Enum('setpoint_source', {'external': 0, 'internal': 1})
RAMP_MODE = Enum('ramp_mode', {'analog': 0, 'digital': 1})

# The two magnet configurations.  'max_current' and 'max_voltage' are the native
# output ranges from the front page of the manual - they are properties of the
# hardware and are used as the hard datatype limits of this node.
#
# 'max_field' is different in kind: the field a given current produces depends
# on the magnet, not on the supply, so there is no hardware number to quote.
# The values here are the manufacturer's example user limits
# (GET_USER_MAX_FIELD_*) and are only a starting point - set 'max_field' on the
# CaylarField module to what your magnet can actually do.  The supply enforces
# its own stored user limit regardless, rejecting anything beyond it.
POWER_LIMITS = {
    '80V140A': {'max_current': 140.0, 'max_voltage': 80.0, 'max_field': 13000.0},
    '90V30A':  {'max_current':  30.0, 'max_voltage': 90.0, 'max_field': 10000.0},
}

# The suffixes that mark a per-configuration command.  Taken from the enum of
# what the front panel can actually be set to, which is the only spelling this
# driver ever *sends*.  What the supply sends back is another matter entirely -
# see CaylarIO.echo_matches.
CONFIG_SUFFIXES = tuple(f'_{member.name}' for member in POWER_CONFIG.members)

# How the supply's error replies map onto SECoP errors.  The reason is the word
# after '..._ERROR' in the reply, e.g. 'SET_CURRENT_ERROR OVERRANGE'.
ERROR_REASONS = {
    'BAD_ARG': (
        BadValueError, 'the supply could not parse the argument'),
    'OVERRANGE': (
        RangeError, "beyond the supply's accepted range"),
    'MAX_CURRENT_OVERRANGE': (
        RangeError, 'beyond the native current range of this power configuration'),
    'MAX_FIELD_OVERRANGE': (
        RangeError, 'beyond the accepted field range (0 - 99999 G)'),
    'RAMP_SPEED_OVERRANGE': (
        RangeError, 'beyond the accepted ramp speed range'),
    'MAINTENANCE_ON': (
        ImpossibleError, 'the supply is in maintenance mode - it ignores all remote '
                         'commands until maintenance is cleared on the front panel'),
    'DEFAULT_ON': (
        ImpossibleError, 'a fault is latched - clear it before powering on'),
    'FIELD_REG': (
        ImpossibleError, 'the supply is regulating on field, not current'),
    'CURRENT_REG': (
        ImpossibleError, 'the supply is regulating on current, not field'),
    'EXTERNAL_SETPOINT_SELEC': (
        ImpossibleError, 'the front panel selector is on the external (analog) '
                         'setpoint - digital setpoints have no effect'),
    'UNKNOWN_PWR_CONFIG': (
        ProgrammingError, 'the supply rejected the power configuration argument'),
}


class CaylarIO(StringIO):
    """The single TCP connection to a Caylar supply.

    Every command is one ASCII line and gets exactly one reply line, so all the
    framing this needs is the '\\n' terminator StringIO already provides.  What
    this class adds is checking that reply: the supply echoes the command back
    with '_OK' appended on success, and with '_ERROR <reason>' on failure.  A
    failure that went unnoticed here would be a command the operator believes
    was applied, so :meth:`execute` and :meth:`query` always check the echo and
    raise rather than return a wrong value.  The check is deliberately loose in
    one respect - see :meth:`echo_matches` for the supply's inconsistency about
    echoing the power configuration suffix.
    """
    end_of_line = '\n'
    default_settings = {'port': 1234}
    # *IDN? answers 'CAYLAR_<serial number>' - manual p.9
    # Has no CAYLAR_ in answer, just <serial number> so will change identifcation for this
    identification = [('*IDN?', '8220_*')]

    power_config = Property(
        """which magnet power configuration the front panel selector is set to

        Checked against the hardware on every connect.  This is a declaration of
        what is expected, not a request: the driver cannot move the selector.
        """,
        EnumType(POWER_CONFIG))

    def checkHWIdent(self):
        """verify identity *and* power configuration on every connect

        StringIO calls this from connectStart(), i.e. on the first connect and
        on every reconnect.  That is exactly where the power configuration
        belongs: if somebody moves the front panel selector and the supply
        reconnects, this node would otherwise carry on enforcing the current
        limits of the other magnet.
        """
        super().checkHWIdent()  # the *IDN? check from `identification`
        actual = int(self.query('POWER_CONFIGURATION')[0])
        if actual != int(self.power_config):
            try:
                actual_name = repr(POWER_CONFIG(actual).name)
            except ValueError:
                actual_name = f'unknown code {actual}'
            self.closeConnection()
            raise ConfigError(
                f'power configuration mismatch: this node is configured for '
                f'{self.power_config.name!r} but the front panel selector is on '
                f'{actual_name}. Either move the selector or fix power_config in '
                f'the cfg file - the current limits enforced here depend on it.')

    @staticmethod
    def echo_matches(echo, name):
        """whether `echo` is this supply's echo of the command `name`

        The supply is not consistent about the power configuration suffix in
        its replies, in two separate ways, both seen on the hardware:

        * it may respell it - 'GET_FIELD_RAMP_SPEED_90V30A' is answered with
          'FIELD_RAMP_SPEED_90V_30A=', with an underscore the command did not
          have;
        * or drop it altogether - 'GET_DIGITAL_CURRENT_RAMP_SPEED_90V30A' is
          answered with 'DIGITAL_CURRENT_RAMP_SPEED='.

        Comparing with underscores removed absorbs the first, and accepting
        the suffix-less base name absorbs the second.  Squashing underscores
        is safe because no two commands in this protocol differ only in where
        their underscores fall, so it cannot make two different commands look
        alike.

        The cost of accepting the short form is that a suffix-less reply does
        not say which configuration it is about, so it cannot be checked
        against the one we asked for.  That is acceptable here: the protocol is
        strictly one reply per command under the io lock, and StringIO flushes
        stale bytes before sending, so a reply arriving for the wrong
        configuration would mean the connection was already out of step.
        """
        squashed = echo.replace('_', '')
        if squashed == name.replace('_', ''):
            return True
        for suffix in CONFIG_SUFFIXES:
            if name.endswith(suffix):
                base = name[:-len(suffix)]
                return squashed == base.replace('_', '')
        return False

    def execute(self, command, *args):
        """send one command, check the '<command>_OK' echo, return the rest

        Returns the fields the supply appended to the echo.  For a SET command
        that is its read-back of what it actually applied, which is worth
        keeping rather than assuming the value went in unchanged.
        """
        request = ' '.join([command, *(f'{arg}' for arg in args)])
        fields = self.communicate(request).split()
        echo = fields[0] if fields else ''
        if echo.endswith('_OK') and self.echo_matches(echo[:-len('_OK')], command):
            return fields[1:]
        self._raise_reply_error(request, fields)
        return None  # unreachable - _raise_reply_error always raises

    def query(self, item):
        """send 'GET_<item>' and return the fields after the '<item>=' echo

        The supply answers a GET by echoing the name *without* the 'GET_' prefix
        and with '=' appended: 'GET_CURRENT' -> 'CURRENT= +12.345678 A'.  For a
        per-configuration item that echo may carry the configuration suffix,
        carry it spelled differently, or omit it - see :meth:`echo_matches`.
        """
        request = f'GET_{item}'
        fields = self.communicate(request).split()
        echo = fields[0] if fields else ''
        if echo.endswith('=') and self.echo_matches(echo[:-1], item):
            return fields[1:]
        self._raise_reply_error(request, fields)
        return None  # unreachable - _raise_reply_error always raises

    def _raise_reply_error(self, request, fields):
        """turn a non-OK reply into the most useful SECoP error we can"""
        if not fields:
            raise CommunicationFailedError(f'no reply to {request!r}')
        if fields[0] == 'WRONGCOMMAND':
            # the supply did not recognise the command at all: a driver bug, or
            # firmware older than the manual this was written against
            raise ProgrammingError(
                f'the supply rejected {request!r} as an unknown command')
        if fields[0].endswith('_ERROR'):
            reason = fields[1] if len(fields) > 1 else ''
            errcls, text = ERROR_REASONS.get(
                reason, (CommunicationFailedError,
                         f'unexpected error reason {reason!r}'))
            raise errcls(f'{request!r} failed: {text}')
        raise CommunicationFailedError(
            f'unexpected reply to {request!r}: {" ".join(fields)!r}')


class CaylarModule(HasIO):
    """common ground for every module talking to a Caylar supply

    Holds nothing but the io class and the per-configuration limits, so the
    three modules below do not each have to work out which magnet they are on.
    """
    ioClass = CaylarIO

    _power_config = None  # '80V140A' or '90V30A'
    _limits = None        # the matching entry of POWER_LIMITS

    def initModule(self):
        super().initModule()
        # power_config is a Property, so reading it costs no communication.
        # That matters here: initModule runs before the server starts, and an
        # unreachable supply must not stop the node coming up.  The check
        # against the hardware happens on connect, in CaylarIO.checkHWIdent.
        self._power_config = self.io.power_config.name
        self._limits = POWER_LIMITS[self._power_config]

    def cfg_command(self, base):
        """the per-configuration form of a command name

        Several commands exist both as '<CMD> <pwr_config> <value>' and as
        '<CMD>_80V140A <value>'.  We always use the suffixed form: it costs
        nothing and it makes the communication log say which magnet a stored
        setting belongs to.
        """
        return f'{base}_{self._power_config}'


class CaylarSupply(CaylarModule, Readable):
    """The power supply itself: power state, faults and housekeeping.

    ``value`` is the output voltage - the one measurement that belongs to the
    supply rather than to a regulation mode.

    This module is also where the current and field modules get their picture of
    the supply from.  It reads the fault, maintenance and power state once per
    poll in :meth:`read_status` and they use the result, so the three modules
    together still make only one set of those queries.
    """
    # 'power off' is a real operating state of a magnet, not a warning and not
    # an error, so extend the standard status codes with DISABLED rather than
    # reporting a permanent WARN whenever the magnet is parked.
    Status = Enum(Readable.Status, DISABLED=StatusType.DISABLED)
    status = Parameter(datatype=StatusType(Status))

    value = Parameter('output voltage', FloatRange(unit='V'))

    power = Parameter('output power state', BoolType(), default=False)
    fault = Parameter('a fault is latched (the protocol calls this "default")',
                      BoolType(), default=False)
    fault_name = Parameter('name of the latched fault, NO_ERROR when there is none',
                           StringType(), default='NO_ERROR')
    maintenance = Parameter('maintenance mode - the supply refuses all remote commands',
                            BoolType(), default=False)

    regulation_mode = Parameter('what the supply is regulating on',
                                EnumType(REGULATION_MODE), default=0)
    setpoint_source = Parameter(
        'where the supply takes its setpoint from (front panel selector)',
        EnumType(SETPOINT_SOURCE), default=1)
    ramp_mode = Parameter(
        """ramp behaviour

        ANALOG is the supply's own fixed maximum slew rate; DIGITAL adds the
        adjustable, slower digital ramp on top of it.  The `ramp` parameters of
        the current and field modules only have an effect in DIGITAL mode.
        """,
        EnumType(RAMP_MODE), readonly=False, default=1)

    box_temp = Parameter('precision electronics temperature',
                         FloatRange(unit='degC'), default=0)
    rack_temp = Parameter('mother board temperature',
                          FloatRange(unit='degC'), default=0)
    water_temp = Parameter('cooling water temperature at the outlet',
                           FloatRange(unit='degC'), default=0)
    water_flow = Parameter('cooling water flow', FloatRange(unit='L/min'), default=0)
    hall_probe_temp = Parameter(
        'Hall probe temperature - watch this settle after a cold start',
        FloatRange(unit='degC'), default=0)

    zero_current = Parameter(
        'current below which the output counts as off, for the power_off guard',
        FloatRange(0, 5, unit='A'), readonly=False, default=0.1)
    power_off_timeout = Parameter(
        'how long power_off waits for the current to come down',
        FloatRange(0, 600, unit='s'), readonly=False, default=120)

    def initModule(self):
        super().initModule()
        vmax = self._limits['max_voltage']
        self.parameters['value'].datatype.set_properties(min=-vmax, max=vmax)

    def read_value(self):
        return float(self.io.query('VOLTAGE')[0])

    def read_status(self):
        """read everything that decides the supply's state, in one place

        The fault, maintenance and power parameters are updated from here rather
        than having their own read_ methods.  That is deliberate: status is
        polled at `pollinterval` while ordinary parameters are polled at the
        much slower `slowinterval`, and a latched fault is not something to
        notice 15 seconds late.  Assigning them here pushes a SECoP update for
        each, so nothing is lost by not polling them separately.
        """
        self.maintenance = bool(int(self.io.query('MAINTENANCE_STATE')[0]))
        self.power = bool(int(self.io.query('POWER_STATE')[0]))
        self.fault = bool(int(self.io.query('DEFAULT_STATE')[0]))
        if self.fault:
            self.fault_name = self.io.query('DEFAULT_NAME')[0]
            return ERROR, f'fault: {self.fault_name}'
        self.fault_name = 'NO_ERROR'

        if self.maintenance:
            # not an error - somebody is deliberately working on the supply -
            # but every SET command is refused until they are done
            return WARN, 'maintenance mode: remote control disabled at the front panel'
        if not self.power:
            return self.Status.DISABLED, 'power off'
        if self.water_flow and self.water_flow < 2.5:
            return WARN, f'cooling water flow low ({self.water_flow:.1f} L/min)'
        return IDLE, ''

    def read_power(self):
        return bool(int(self.io.query('POWER_STATE')[0]))

    def read_regulation_mode(self):
        return self.io.query('REGUL_MODE')[0].lower()

    def read_setpoint_source(self):
        # GET_CMD_SELEC: 0 = external (analog) setpoint, 1 = internal (digital)
        return int(self.io.query('CMD_SELEC')[0])

    def read_ramp_mode(self):
        return self.io.query('RAMP_MODE')[0].lower()

    def write_ramp_mode(self, value):
        return self.io.execute('SET_RAMP_MODE', RAMP_MODE(value).name.upper())[0].lower()

    def read_box_temp(self):
        return float(self.io.query('BOX_TEMP')[0])

    def read_rack_temp(self):
        return float(self.io.query('RACK_TEMP')[0])

    def read_water_temp(self):
        return float(self.io.query('WATER_TEMP')[0])

    def read_water_flow(self):
        # below the sensor's threshold the supply answers '<2.5' rather than a
        # number, so this cannot just be float()ed
        return float(self.io.query('WATER_FLOW')[0].lstrip('<'))

    def read_hall_probe_temp(self):
        return float(self.io.query('HALL_PROBE_TEMP')[0])

    def measured_current(self):
        """the measured output current, for the power_off guard

        Not a parameter of this module - current belongs to CaylarCurrent.  This
        is here so the guard does not depend on that module being configured, or
        on how recently it happens to have been polled.
        """
        return float(self.io.query('CURRENT')[0])

    @Command()
    def clear_fault(self):
        """clear a latched fault (the reset button on the front panel)

        Has no effect while the condition that caused the fault is still there.
        """
        self.io.execute('CLEAR_DEFAULT')
        self.read_status()

    @Command()
    def user_fault_on(self):
        """assert a user fault, which cuts the power

        This is the supply's remote interlock (SET_DEFAULT_ON): it trips the
        supply exactly as a real fault would, so that another instrument or
        program can stop the magnet. It cuts the output instantly and therefore
        carries the same risk to an inductive load as `force_power_off` - it is
        an interlock, not a way to turn the magnet off tidily.

        Clearing it takes two steps, because the supply distinguishes the
        condition from the latch: `user_fault_off` releases the condition, then
        `clear_fault` clears the latch.
        """
        self.io.execute('SET_DEFAULT_ON')
        self.read_status()

    @Command()
    def user_fault_off(self):
        """release the user fault asserted by `user_fault_on`

        The supply stays faulted after this until `clear_fault`.
        """
        self.io.execute('SET_DEFAULT_OFF')
        self.read_status()

    @Command()
    def power_on(self):
        """switch the output on"""
        self.io.execute('SET_POWER_ON')
        self.read_status()

    @Command()
    def power_off(self):
        """ramp the output down, then switch the power off

        SET_POWER_OFF cuts the output instantly, whatever the supply happens to
        be delivering at the time.  Into an electromagnet that is the case the
        manual warns about in red: the energy stored in the inductance has
        nowhere to go and can damage the supply despite its freewheel diodes.

        So this zeroes the setpoint, waits for the *measured* current to
        actually come down, and only then cuts the power.  It blocks while it
        waits - up to `power_off_timeout` - and raises rather than cutting the
        power if the current never gets there.  Use `force_power_off` only when
        you know the output is already at zero.
        """
        if self.read_setpoint_source() == SETPOINT_SOURCE.external:
            raise ImpossibleError(
                'the front panel selector is on the external (analog) setpoint, '
                'so this driver cannot ramp the output down. Wind the setpoint '
                'down at its source, then use force_power_off.')

        # zero the setpoint in whichever mode the supply is regulating on
        if self.read_regulation_mode() == REGULATION_MODE.field:
            self.io.execute('SET_FIELD', 0)
        else:
            self.io.execute('SET_CURRENT', 0)

        deadline = time.monotonic() + self.power_off_timeout
        while True:
            current = self.measured_current()
            if abs(current) <= self.zero_current:
                break
            if time.monotonic() > deadline:
                raise ImpossibleError(
                    f'current is still {current:+.3f} A after '
                    f'{self.power_off_timeout:.0f} s - not cutting the power. '
                    f'Check the ramp speed and the setpoint source.')
            time.sleep(0.5)

        self.io.execute('SET_POWER_OFF')
        self.read_status()

    @Command()
    def force_power_off(self):
        """switch the power off immediately, with no ramp and no check

        Only for when the output is already known to be at zero, or when the
        supply has to be stopped regardless of the consequences.  See
        `power_off` for what those consequences are.
        """
        self.log.warning('forced power off at %+.3f A', self.measured_current())
        self.io.execute('SET_POWER_OFF')
        self.read_status()

    @Command()
    def set_regulation_mode_current(self):
        """put the supply into current regulation mode"""
        self.io.execute('SET_REGUL_CURRENT')
        self.read_regulation_mode()
        self.read_status()

    @Command()
    def set_regulation_mode_field(self):
        """put the supply into field regulation mode"""
        self.io.execute('SET_REGUL_FIELD')
        self.read_regulation_mode()
        self.read_status()


class CaylarRegulation(CaylarModule, Drivable):
    """shared behaviour of the two regulation modes

    Current and field regulation differ in their units, their setpoint commands
    and their ramp commands, but they read their status the same way and both
    have to put the supply into their own regulation mode before a setpoint
    means anything.  That is all this class holds - everything unit-specific is
    written out in the two subclasses below.
    """
    Status = Enum(Drivable.Status, DISABLED=StatusType.DISABLED)
    status = Parameter(datatype=StatusType(Status))

    supply = Attached(CaylarSupply, 'the supply module, for power and fault state')

    #: which REGULATION_MODE this module needs the supply to be in
    regulation_mode = None
    #: protocol name of the setpoint command: 'CURRENT' -> SET_CURRENT
    setpoint_command = None

    def read_status(self):
        """status of this regulation mode

        Anything wrong with the supply itself outranks anything this module
        could say, so that comes first and passes straight through.  After that:
        a setpoint that cannot reach the hardware is a warning, being on the
        other regulation mode is a warning, and only then does it matter whether
        we are ramping.
        """
        # Refresh the target from the hardware before deciding anything else.
        # The setpoint can be changed at the front panel at any time, and a
        # SECoP target that quietly disagreed with what the operator typed
        # there would be worse than no target at all.  It is done ahead of
        # every early return below, so the target keeps tracking even while the
        # supply is off, faulted, or regulating on the other quantity - those
        # are exactly the states somebody is most likely to be standing at the
        # front panel changing things.  It costs no extra communication: the
        # ramping test needs the setpoint anyway.
        target = self.read_target()

        code, text = self.supply.status
        if code >= ERROR or code == self.Status.DISABLED or self.supply.maintenance:
            return code, text

        if self.supply.setpoint_source == SETPOINT_SOURCE.external:
            return WARN, ('external (analog) setpoint selected - '
                          'targets set here have no effect')
        if self.supply.regulation_mode != self.regulation_mode:
            return WARN, f'supply is regulating on {self.supply.regulation_mode.name}'

        if self.is_ramping(target):
            return BUSY, 'ramping'
        if code >= WARN:
            # e.g. low cooling water flow: keep the supply's warning visible
            return code, text
        return IDLE, ''

    def is_ramping(self, target):
        """whether the output is still on its way to `target`

        GET_DIGITAL_RAMP_STATE is the authoritative answer, but only in DIGITAL
        ramp mode - in ANALOG mode it always reads 0, because the analog ramp is
        not something the firmware tracks.  So fall back on comparing the value
        with the setpoint against `tolerance` in that case.

        Takes the setpoint as an argument rather than reading it, so that
        read_status reads it exactly once per poll.
        """
        if self.supply.ramp_mode == RAMP_MODE.digital:
            if int(self.io.query('DIGITAL_RAMP_STATE')[0]):
                return True
        return abs(self.value - target) > self.tolerance

    def check_regulation_mode(self):
        """put the supply into this module's regulation mode, if it is not already

        Checked before switching rather than switched unconditionally: switching
        is not free, and SET_REGUL_FIELD is refused outright when the external
        setpoint is selected.
        """
        if self.supply.read_regulation_mode() != self.regulation_mode:
            raise ImpossibleError(f'Wrong regulation mode. Power supply is in \
                                  {self.supply.read_regulation_mode().name} mode but command ' \
                                  f'assumes {self.regulation_mode.name} mode. Change regulation \
                                    mode before continuing.')

    def write_target(self, target):
        self.check_regulation_mode()
        if not self.supply.read_power():
            # Not an error: the supply can be told (DIVERS menu, CMD UPDATE =
            # PON/OFF) to store a setpoint received while off and apply it at
            # the next power on, and pre-setting before powering up is a
            # reasonable thing to do.  But if that option is off the setpoint is
            # silently discarded, which is worth a line in the log.
            self.log.warning('target set while the power is off - it will only take '
                             'effect if the supply is set to store setpoints')
        applied = float(self.io.execute(f'SET_{self.setpoint_command}', target)[0])
        # setting status here, before returning, is what makes the SECoP client
        # see BUSY straight away rather than at the next poll
        self.status = BUSY, 'target changed'
        return applied

    def stop(self):
        """hold at the present value

        There is no stop command in the protocol, so stopping means setting the
        target to wherever the output has got to.
        """
        self.write_target(self.read_value())


class CaylarCurrent(CaylarRegulation):
    """Current regulation: the magnet current in amps.

    This is the supply's native mode, and the one to use unless you specifically
    want the Hall probe in the loop.  The limits come from the power
    configuration declared on the io module.
    """
    regulation_mode = REGULATION_MODE.current
    setpoint_command = 'CURRENT'

    value = Parameter('measured output current', FloatRange(unit='A'))
    target = Parameter('current setpoint, tracking the supply', FloatRange(unit='A'))
    tolerance = Parameter('how close to the setpoint counts as arrived',
                          FloatRange(0, unit='A'), readonly=False, default=0.05)

    ramp = Parameter(
        """digital ramp speed for this power configuration

        Only has an effect when the supply's `ramp_mode` is DIGITAL.  Stored
        permanently in the supply, per power configuration.
        """,
        FloatRange(0.01, 99.9, unit='A/s'), readonly=False, default=1.0)
    analog_ramp = Parameter(
        "the supply's fixed analog ramp speed - set at Caylar, read only",
        FloatRange(unit='A/s'), default=0)
    user_max = Parameter(
        "the supply's own stored current limit for this power configuration",
        FloatRange(0, unit='A'), readonly=False, default=0)

    def initModule(self):
        super().initModule()
        imax = self._limits['max_current']
        for pname in ('value', 'target'):
            self.parameters[pname].datatype.set_properties(min=-imax, max=imax)
        self.parameters['user_max'].datatype.set_properties(min=0, max=imax)

    def read_value(self):
        return float(self.io.query('CURRENT')[0])

    @nopoll  # read once per status poll instead - see CaylarRegulation.read_status
    def read_target(self):
        """the setpoint the supply is holding, whoever put it there

        GET_CURRENT_SETPOINT returns the last digital setpoint the supply
        received, whether that came from this driver or from somebody at the
        front panel keyboard.  Reading it is what keeps `target` honest.
        """
        return float(self.io.query('CURRENT_SETPOINT')[0])

    def read_ramp(self):
        return float(self.io.query(self.cfg_command('DIGITAL_CURRENT_RAMP_SPEED'))[0])

    def write_ramp(self, value):
        return float(self.io.execute(
            f'SET_{self.cfg_command("DIGITAL_CURRENT_RAMP_SPEED")}', value)[0])

    def read_analog_ramp(self):
        return float(self.io.query(self.cfg_command('ANALOG_CURRENT_RAMP_SPEED'))[0])

    def read_user_max(self):
        return float(self.io.query(self.cfg_command('USER_MAX_CURRENT'))[0])

    def write_user_max(self, value):
        return float(self.io.execute(
            f'SET_{self.cfg_command("USER_MAX_CURRENT")}', value)[0])


class CaylarField(CaylarRegulation):
    """Field regulation: the magnet field in gauss, closed loop on the Hall probe.

    Gauss is the unit the protocol works in; convert to tesla downstream if that
    is what the instrument wants.

    Field regulation only works with the internal (digital) setpoint selected on
    the front panel - the supply refuses SET_REGUL_FIELD otherwise, which
    surfaces here as a clear error.

    Before using this for the first time on a given magnet, check the Hall probe
    orientation: in current regulation, a positive current must read as a
    positive field.  An inverted probe makes the regulation drive the current
    the wrong way into saturation, which trips the supply's software protection.
    """
    regulation_mode = REGULATION_MODE.field
    setpoint_command = 'FIELD'

    value = Parameter('measured field', FloatRange(unit='G'))
    target = Parameter('field setpoint, tracking the supply', FloatRange(unit='G'))
    tolerance = Parameter('how close to the setpoint counts as arrived',
                          FloatRange(0, unit='G'), readonly=False, default=1.0)

    max_field = Property(
        """largest field this magnet can be asked for, in gauss

        Sets the hard limits of value, target and setpoint.  Left at 0 it falls
        back to the manufacturer's example limit for the power configuration,
        which is a starting point rather than a property of your magnet - set it.
        """,
        FloatRange(0, 99999), default=0)

    ramp = Parameter(
        """field setpoint ramp speed for this power configuration

        Stored permanently in the supply, per power configuration.
        """,
        FloatRange(0.01, 9999.99, unit='G/s'), readonly=False, default=10.0)
    user_max = Parameter(
        "the supply's own stored field limit for this power configuration",
        FloatRange(0, 99999, unit='G'), readonly=False, default=0)
    regulating = Parameter(
        'whether the field regulation loop is running (it can be paused)',
        BoolType(), readonly=False, default=True)

    def initModule(self):
        super().initModule()
        bmax = self.max_field or self._limits['max_field']
        for pname in ('value', 'target'):
            self.parameters[pname].datatype.set_properties(min=-bmax, max=bmax)
        self.parameters['user_max'].datatype.set_properties(min=0, max=bmax)

    def read_value(self):
        return float(self.io.query('FIELD')[0])

    @nopoll  # read once per status poll instead - see CaylarRegulation.read_status
    def read_target(self):
        """the field setpoint the supply is holding, whoever put it there

        See CaylarCurrent.read_target - GET_FIELD_SETPOINT behaves the same way.
        """
        return float(self.io.query('FIELD_SETPOINT')[0])

    def read_ramp(self):
        return float(self.io.query(self.cfg_command('FIELD_RAMP_SPEED'))[0])

    def write_ramp(self, value):
        return float(self.io.execute(
            f'SET_{self.cfg_command("FIELD_RAMP_SPEED")}', value)[0])

    def read_user_max(self):
        return float(self.io.query(self.cfg_command('USER_MAX_FIELD'))[0])

    def write_user_max(self, value):
        return float(self.io.execute(
            f'SET_{self.cfg_command("USER_MAX_FIELD")}', value)[0])

    def read_regulating(self):
        return bool(int(self.io.query('FIELD_REGUL_STATE')[0]))

    def write_regulating(self, value):
        self.io.execute('SET_FIELD_REGUL_RUN' if value else 'SET_FIELD_REGUL_PAUSE')
        return value
