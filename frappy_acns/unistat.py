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
"""Huber temperature control units (Unistat / Pilot ONE) over the PB protocol.

Reference: *Huber Data Communication Manual PB*, V2.8.0en / 11.12.25.

The PB ("Process Bus") protocol is ASCII, master/slave, one variable per
exchange, and every command is exactly ten characters::

    {mttvvvv<CR><LF>

    {     start character (0x7B)
    m     'M' for a master request, 'S' for the thermostat's response
    tt    address in the variable table, 2 hex characters
    vvvv  value, 4 hex characters; "****" in a request means
          "tell me the value, do not change it"

There is no closing brace and no checksum. The thermostat answers every
syntactically valid request, echoing the address; a malformed request gets *no
reply at all*, so protocol mistakes surface as timeouts rather than as errors.

Layering:

- :class:`UnistatIO` is the transport: line framing and the connect-time
  identification only.
- :class:`UnistatMixin` owns the PB framing - building requests, checking the
  echoed address, and converting raw 16-bit fields to physical values. It lives
  on the module side rather than on the IO class so that a protocol-level
  simulator (:mod:`frappy_acns.unistat_claude_sim`) can stand in for the IO
  unchanged, which is the same split :mod:`frappy_acns.pfeiffer_new` uses.
- :class:`Unistat405` is the main temperature module, and
  :class:`UnistatSensor` exposes any other read-only table entry.

Two things about this device that are easy to get wrong, both handled here:

1. Temperatures are *not* plain two's complement - see :func:`decode_temperature`.
2. Values written over PB are **not** stored permanently. They revert to the
   last manually entered value after any interruption of the power supply.
"""

import time
from dataclasses import dataclass

from frappy.core import BUSY, BoolType, Command, Drivable, EnumType, ERROR, \
    FloatRange, HasIO, IDLE, IntRange, Parameter, Property, Readable, \
    StringIO, StringType, WARN, nopoll
from frappy.errors import CommunicationFailedError, ImpossibleError, \
    RangeError
from frappy_psi.convergence import HasConvergence


# ==============================================================================
# PB value encoding
# ==============================================================================

#: Reply value for an address that is undefined, or not released for the
#: device's E-grade licence. The manual: "If an address is contacted which is
#: not defined or not released (cf. E-grade), the command is answered with the
#: same address and the value 7FFF."
NOT_RELEASED = 0x7FFF

#: Lowest representable temperature, in LSB units of 0.01 degC.
TEMP_MIN_RAW = -15111

#: "Temperature not available", raw 0xC504. The manual: "The response for
#: temperatures that are not available is -151.00 degC." Note this is distinct
#: from TEMP_MIN_RAW - see decode_temperature for why that matters.
TEMP_UNAVAILABLE_RAW = -15100


def decode_signed(raw):
    """Plain 16-bit two's complement."""
    return raw - 0x10000 if raw >= 0x8000 else raw


def decode_unsigned(raw):
    """Identity - the field is already an unsigned 16-bit integer."""
    return raw


def decode_temperature(raw):
    """Huber's 16-bit temperature encoding, which is *not* two's complement.

    From the manual's notes on the command table:

        If a value range is specified between -15111 and 50000, the number is
        to be interpreted as a signed number. However, if the decimal number is
        smaller than -15111, it must be interpreted as an unsigned number.

    That is how -151.11 ... +500.00 degC is squeezed into 16 bits: the codes a
    signed reading could never legitimately produce (below -151.11 degC) are
    reused for the top of the range, so 0x8000...0xC4F8 mean +327.68...+504.24
    degC rather than large negative numbers.

    The rule dovetails exactly with the "not available" sentinel: 0xC504 decodes
    signed to -15100, which is *not* below -15111, so it stays signed and
    remains recognisable; 0xC4F8 decodes to -15112, which is, so it flips.
    """
    value = decode_signed(raw)
    if value < TEMP_MIN_RAW:
        return raw  # re-read as unsigned
    return value


def encode_raw(raw):
    """Render a raw integer as the 4-character hex field of a PB command.

    Masking to 16 bits gives two's complement for negatives and passes the
    above-32767 temperature codes through untouched, so one encoder serves
    every variable type.
    """
    return f'{raw & 0xFFFF:04X}'


DECODERS = {
    'temperature': decode_temperature,
    'signed': decode_signed,
    'unsigned': decode_unsigned,
    'bits': decode_unsigned,
}


@dataclass(frozen=True)
class PBVar:
    """One entry of the PB variable table.

    :param address: table place, the ``tt`` field of the command
    :param lsb: physical value per raw count (the manual's "LSB" column)
    :param kind: which decoder applies - see :data:`DECODERS`
    :param unit: for documentation and for the SECoP datatype
    """
    address: int
    lsb: float = 1.0
    kind: str = 'signed'
    unit: str = ''

    def decode(self, raw):
        value = DECODERS[self.kind](raw)
        return value * self.lsb if self.lsb != 1.0 else value

    def encode(self, value):
        return encode_raw(int(round(value / self.lsb)))


#: The subset of the PB table this module uses. The "E-grade" comment on each
#: line is the licence level the manual says it needs - anything above Basic may
#: answer NOT_RELEASED on a given unit. See Unistat405._probe_optional.
PB_VARS = {
    'setpoint':        PBVar(0x00, 0.01, 'temperature', 'degC'),  # Basic
    'internal_temp':   PBVar(0x01, 0.01, 'temperature', 'degC'),  # Basic
    'return_temp':     PBVar(0x02, 0.01, 'temperature', 'degC'),  # Explore
    'pump_pressure':   PBVar(0x03, 1.0, 'unsigned', 'mbar'),      # Basic
    'power':           PBVar(0x04, 1.0, 'signed', 'W'),           # Explore
    'error_number':    PBVar(0x05, 1.0, 'signed'),                # Basic
    'warning_number':  PBVar(0x06, 1.0, 'signed'),                # Basic
    'process_temp':    PBVar(0x07, 0.01, 'temperature', 'degC'),  # Basic
    'status1':         PBVar(0x0A, 1.0, 'bits'),                  # Basic
    'level':           PBVar(0x0F, 1.0, 'signed'),                # Basic
    'control_mode':    PBVar(0x13, 1.0, 'unsigned'),              # Exclusive
    'control_active':  PBVar(0x14, 1.0, 'unsigned'),              # Basic
    'circulation':     PBVar(0x16, 1.0, 'unsigned'),              # Basic
}


# ==============================================================================
# vStatus1 (0x0A) bit field
# ==============================================================================

class Status1:
    """Bit positions in vStatus1. Only the ones this driver acts on."""
    CONTROL_ACTIVE = 0   # temperature control operating mode
    CIRCULATION = 1
    COMPRESSOR = 2
    PROCESS_MODE = 3     # temperature control mode "process control"
    PUMP = 4
    COOLING_AVAILABLE = 5
    KEYLOCK = 6
    AUTO_PID = 7
    ERROR = 8
    WARNING = 9
    INTERNAL_SETMODE = 10
    EXTERNAL_SETMODE = 11
    DV_EGRADE = 12
    #: Reads 0 on the very first read after a restart of the electronics and 1
    #: on every read after that, so a 0 here means the unit rebooted since we
    #: last looked - see Unistat405.read_status.
    NO_RESTART = 14
    FREEZE_PROTECTION = 15


class ControlMode:
    """Values of vTmpMode (0x13), and of vStatus1 bit 3."""
    INTERNAL = 0   # controller works on the internal (bath) sensor
    PROCESS = 1    # cascade control on the process Pt100


CONTROL_MODE = EnumType('control_mode', internal=ControlMode.INTERNAL,
                        process=ControlMode.PROCESS)

CONTROL_MODE_NAMES = {ControlMode.INTERNAL: 'internal',
                      ControlMode.PROCESS: 'process'}


# ==============================================================================
# SECoP modules
# ==============================================================================

class UnistatIO(StringIO):
    """Transport for the PB protocol.

    Deliberately thin: the framing lives in :class:`UnistatMixin` so that the
    simulator can replace this class without reimplementing it.
    """
    end_of_line = '\r\n'
    #: The manual quotes a typical response time below 300 ms but recommends
    #: "waiting at least a second for the response in order not to risk
    #: unnecessary timeouts", and says Com.G@te / Web.G@te may be slower still.
    timeout = 2.0
    #: A vStatus1 query is the cheapest thing that proves we are talking to a
    #: Pilot ONE and not, say, a terminal server with nothing behind it.
    identification = [('{M0A****', r'\{S0A[0-9A-Fa-f]{4}')]


class UnistatMixin(HasIO):
    """PB framing: request construction, reply validation, value conversion."""
    ioClass = UnistatIO

    def _transact(self, address, field):
        """Send one PB command and return the raw 16-bit reply value.

        :param address: table place
        :param field: the 4-character value field, or '****' to query
        :return: the reply's value field as an unsigned 16-bit int
        """
        # No end_of_line here: StringIO.communicate appends it. Adding it by
        # hand would send the terminator twice and the command would be
        # ignored with no reply at all.
        reply = self.communicate(f'{{M{address:02X}{field}')
        if len(reply) < 8 or not reply.startswith('{S'):
            raise CommunicationFailedError(
                f'malformed PB reply {reply!r} to address 0x{address:02X}')
        try:
            echoed = int(reply[2:4], 16)
            raw = int(reply[4:8], 16)
        except ValueError:
            raise CommunicationFailedError(
                f'non-hex PB reply {reply!r}') from None
        if echoed != address:
            # The stream has desynchronised - our reply belongs to an earlier
            # request. Better to say so than to report another variable's value.
            raise CommunicationFailedError(
                f'PB reply for address 0x{echoed:02X}, expected '
                f'0x{address:02X} (reply {reply!r})')
        return raw

    def pb_get_raw(self, address):
        """Query one address. Returns None if it answered NOT_RELEASED."""
        raw = self._transact(address, '****')
        return None if raw == NOT_RELEASED else raw

    def pb_set_raw(self, address, raw):
        """Set one address. Returns the echoed value, or None if NOT_RELEASED."""
        echoed = self._transact(address, encode_raw(raw))
        return None if echoed == NOT_RELEASED else echoed

    def pb_get(self, name):
        """Query a named variable and decode it, or None if not released."""
        var = PB_VARS[name]
        raw = self.pb_get_raw(var.address)
        return None if raw is None else var.decode(raw)

    def pb_set(self, name, value):
        """Set a named variable and decode the echoed value."""
        var = PB_VARS[name]
        echoed = self._transact(var.address, var.encode(value))
        if echoed == NOT_RELEASED:
            raise ImpossibleError(
                f'{name} (PB address 0x{var.address:02X}) is not defined or '
                f'not released on this device (answered 7FFF) - this variable '
                f'may need a higher E-grade licence')
        return var.decode(echoed)


class Unistat405(HasConvergence, UnistatMixin, Drivable):
    """A Huber Unistat as a single temperature-control module.

    ``value`` mirrors whichever temperature the device is actually controlling
    on: the internal (bath/flow/jacket) sensor in internal mode, the process
    Pt100 on the LEMOSA connector in process (cascade) mode. Both are always
    published separately as well, so nothing is hidden by that choice.

    Note that nothing this module writes is stored permanently in the
    thermostat - the manual is explicit that settings changed by a data command
    "will be reset to the last manually entered value after any interruption in
    the power supply".
    """
    value = Parameter('controlled temperature', FloatRange(unit='degC'))
    target = Parameter('setpoint for the temperature controller',
                       FloatRange(-151.11, 500.0, unit='degC'), default=25)

    internal_temp = Parameter(
        'internal temperature - the fluid flowing to the application, also '
        'called the flow, jacket or bath temperature',
        FloatRange(unit='degC'), readonly=True, default=0)
    process_temp = Parameter(
        'process temperature measured by the Pt100 on the LEMOSA connector '
        '(reads -151 degC when no sensor is connected)',
        FloatRange(unit='degC'), readonly=True, default=0)

    control_mode = Parameter(
        'which sensor the controller works on', CONTROL_MODE,
        readonly=False, default=0)
    control_active = Parameter(
        'whether temperature control is running', BoolType(),
        readonly=False, default=False)

    error_number = Parameter('Huber error number, 0 when there is none',
                             IntRange(-32768, 32767), readonly=True, default=0,
                             group='diagnostics')
    warning_number = Parameter('Huber warning number, 0 when there is none',
                               IntRange(-32768, 32767), readonly=True,
                               default=0, group='diagnostics')
    status_bits = Parameter('raw vStatus1 bit field', IntRange(0, 0xFFFF),
                            readonly=True, default=0, group='diagnostics',
                            visibility='expert')

    control_mode_default = Property(
        'control mode to assume when vTmpMode (PB 0x13) is not released on '
        'this device - it needs the "Exclusive" E-grade, so on many units it '
        'cannot be read and this is what value follows instead',
        datatype=CONTROL_MODE, default=0)
    compressor_min_runtime = Parameter(
        'minimum time the compressor must have been running before '
        'temperature control may be switched off. The manual requires 5 '
        'minutes; set to 0 to disable the check',
        FloatRange(0, unit='s'), default=300.0, readonly=False, group='tuning')

    pollinterval = Parameter(default=5)

    #: monotonic time when vStatus1 bit 5 was first seen set, else None
    _cooling_since = None
    #: False once we have found out vTmpMode is not readable on this device
    _mode_readable = True
    #: last seen state of the "no restart" bit, to report a reboot once
    _seen_running = False

    # -- lifecycle ---------------------------------------------------------
    def initModule(self):
        super().initModule()
        self._probe_optional()

    def _probe_optional(self):
        """Find out once whether vTmpMode is released on this device.

        vTmpMode (0x13) needs the "Exclusive" E-grade. When it is not
        available the device answers 7FFF, and rather than failing every poll
        we fall back to control_mode_default and say so once.
        """
        try:
            mode = self.pb_get('control_mode')
        except Exception as e:  # pylint: disable=broad-except
            # Probing must never stop the module from starting - the device may
            # simply not be reachable yet, and the poller will retry.
            self.log.warning('could not probe vTmpMode: %r', e)
            return
        if mode is None:
            self._mode_readable = False
            self.control_mode = int(self.control_mode_default)
            self.log.warning(
                'vTmpMode (PB 0x13) is not released on this device (needs the '
                '"Exclusive" E-grade); assuming %r. Set control_mode_default '
                'in the config if that is wrong.',
                CONTROL_MODE_NAMES[int(self.control_mode_default)])

    # -- polling -----------------------------------------------------------
    def doPoll(self):
        """One ordered pass over the handful of variables we need.

        PB allows only one variable per command, so each of these is its own
        round trip. They are done here, in order, rather than from individual
        pollable read_* methods so that `value` is derived from a temperature
        read in the same cycle as the mode it was chosen by.
        """
        bits = self.read_status_bits()
        self.control_active = bool(bits & (1 << Status1.CONTROL_ACTIVE))
        if self._mode_readable:
            self.read_control_mode()
        self.internal_temp = self.pb_get('internal_temp')
        process = self.pb_get('process_temp')
        if process is not None:
            self.process_temp = process
        self.read_target()
        self.value = (self.process_temp if self.control_mode == ControlMode.PROCESS
                      else self.internal_temp)
        self.read_status()
        super().doPoll()  # lets HasConvergence cycle its state machine

    @nopoll
    def read_status_bits(self):
        bits = self.pb_get('status1')
        if bits is None:
            raise CommunicationFailedError('vStatus1 (PB 0x0A) is not released')
        bits = int(bits)
        self.status_bits = bits
        # Track how long cooling power has been available, for the compressor
        # minimum-runtime rule enforced in write_control_active.
        if bits & (1 << Status1.COOLING_AVAILABLE):
            if self._cooling_since is None:
                self._cooling_since = time.monotonic()
        else:
            self._cooling_since = None
        return bits

    @nopoll
    def read_value(self):
        name = ('process_temp' if self.control_mode == ControlMode.PROCESS
                else 'internal_temp')
        return self.pb_get(name)

    @nopoll
    def read_internal_temp(self):
        return self.pb_get('internal_temp')

    @nopoll
    def read_process_temp(self):
        return self.pb_get('process_temp')

    @nopoll
    def read_target(self):
        return self.pb_get('setpoint')

    @nopoll
    def read_control_mode(self):
        mode = self.pb_get('control_mode')
        if mode is None:
            # Released a moment ago, not now - treat as the configured default
            # rather than letting a 7FFF become a mode number.
            self._mode_readable = False
            return int(self.control_mode_default)
        return int(mode)

    @nopoll
    def read_control_active(self):
        return bool(self.pb_get('control_active'))

    @nopoll
    def read_error_number(self):
        return int(self.pb_get('error_number') or 0)

    @nopoll
    def read_warning_number(self):
        return int(self.pb_get('warning_number') or 0)

    # -- driving -----------------------------------------------------------
    def write_target(self, value):
        echoed = self.pb_set('setpoint', value)
        self.convergence_start()
        return echoed

    def write_control_mode(self, value):
        if not self._mode_readable:
            raise ImpossibleError(
                'vTmpMode (PB 0x13) is not released on this device - the '
                'control mode can only be changed at the Pilot')
        return int(self.pb_set('control_mode', int(value)))

    def write_control_active(self, value):
        if not value:
            self._check_compressor_runtime()
        return bool(self.pb_set('control_active', 1 if value else 0))

    def _check_compressor_runtime(self):
        """Refuse to stop temperature control too soon after the compressor started.

        The manual, under vTmpActive: "To stop the temperature control
        properly, the compressor must have been switched on for at least 5
        minutes. To this end, bit 5 of the PB command vStatus1 can be
        evaluated. Only when this bit has been set to 1 for at least 5 minutes
        has the minimum runtime of the compressor been fulfilled."
        """
        if not self.compressor_min_runtime or self._cooling_since is None:
            return  # check disabled, or the compressor is not running
        elapsed = time.monotonic() - self._cooling_since
        if elapsed < self.compressor_min_runtime:
            raise ImpossibleError(
                f'the compressor has only been running for {elapsed:.0f} s; '
                f'stopping temperature control needs '
                f'{self.compressor_min_runtime:.0f} s to protect it. Retry in '
                f'{self.compressor_min_runtime - elapsed:.0f} s, or set '
                f'compressor_min_runtime to 0 to override.')

    @Command
    def stop(self):
        """Hold at the current temperature rather than switching control off.

        Deliberately does NOT switch temperature control off: that is what
        control_active is for, and doing it here would run into the
        compressor's minimum runtime on what a client expects to be a
        always-available command.
        """
        self.write_target(self.value)

    @Command(group='diagnostics')
    def clear_messages(self):
        """Clear the pending error and confirm the current warning.

        Sending the value 1 to vError / vWarn is how the manual says to delete
        existing messages.
        """
        self.pb_set('error_number', 1)
        self.pb_set('warning_number', 1)
        self.read_status()

    # -- status ------------------------------------------------------------
    def read_status(self):
        """Device faults first, then the convergence state machine.

        Ordering matters: an error or warning from the thermostat is more
        important than whether we happen to be within tolerance, and "control
        is switched off" has to beat BUSY, or the module would sit at
        "approaching setpoint" forever on a device that is not heating.
        """
        bits = self.status_bits
        if bits & (1 << Status1.ERROR):
            self.read_error_number()
            return ERROR, f'device error {self.error_number}'
        if bits & (1 << Status1.WARNING):
            self.read_warning_number()
            return WARN, f'device warning {self.warning_number}'
        if not bits & (1 << Status1.NO_RESTART) and self._seen_running:
            return WARN, ('the thermostat restarted - settings written over PB '
                          'are not retained across a power interruption')
        self._seen_running = True
        if not bits & (1 << Status1.CONTROL_ACTIVE):
            return IDLE, 'temperature control off'
        return HasConvergence.read_status(self)


class UnistatSensor(UnistatMixin, Readable):
    """Any read-only entry of the PB table, as its own module.

    Used for the secondary quantities - pump pressure, power, return
    temperature, fill level - so that each is a short config entry rather than
    a class. Configure `variable` with a name from :data:`PB_VARS`, or give
    `address`/`lsb`/`kind` directly for a variable not in that table.
    """
    value = Parameter('the variable value', FloatRange())

    variable = Property('name of the variable in PB_VARS, or empty to use '
                        'address/lsb/kind', StringType(), default='')
    address = Property('PB table address, when variable is not given',
                       IntRange(0, 0xFF), default=0)
    lsb = Property('physical value per raw count, when variable is not given',
                   FloatRange(), default=1.0)
    kind = Property('decoder, when variable is not given',
                    StringType(), default='signed')

    _var = None

    def initModule(self):
        super().initModule()
        if self.variable:
            try:
                self._var = PB_VARS[self.variable]
            except KeyError:
                raise RangeError(
                    f'unknown PB variable {self.variable!r}; known names are '
                    f'{", ".join(sorted(PB_VARS))}') from None
        else:
            self._var = PBVar(self.address, self.lsb, self.kind)

    def read_value(self):
        raw = self.pb_get_raw(self._var.address)
        if raw is None:
            self.status = WARN, (
                f'PB address 0x{self._var.address:02X} is not defined or not '
                f'released on this device (answered 7FFF)')
            return self.value
        self.status = IDLE, ''
        return self._var.decode(raw)

    def read_status(self):
        return self.status
