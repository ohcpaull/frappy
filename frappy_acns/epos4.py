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
"""Maxon EPOS4 positioning controllers reached over ModbusTCP.

The drives speak CANopen and sit behind a gateway (e.g. a Moxa Mport 5121)
that exposes each drive's CiA-402 objects as a contiguous window of Modbus
registers. Addresses in :data:`TAG_MAP` are *relative* to that window, so
several drives behind one gateway share the table and are told apart by the
``input_offset`` / ``holding_offset`` properties of each module.

Layering:

- :mod:`frappy_acns.modbusTCP` is the transport.
- :class:`EPOS4Base` is the register windowing, block read and CiA-402 status
  decoding shared by every EPOS4, regardless of how it is driven.
- :class:`EPOS4Motion` adds the CiA-402 enable/disable/fault state machine
  that is likewise shared, parameterised by ``OPERATING_MODE``.
- :class:`EPOS4Velocity` and :class:`EPOS4Position` are the two concrete
  motion profiles currently supported: continuous rotation commanded through
  an analog output voltage (Cyclic Synchronous Velocity), and point-to-point
  moves (Profile Position Mode). Configure a device with whichever one
  matches how that axis is wired and used.
- Instrument-specific modules (e.g. :mod:`frappy_acns.tumbler`) subclass
  whichever of the two matches the axis, adding only labels/limits.
"""

import time
from enum import IntEnum, IntFlag

from frappy.core import BUSY, Command, Drivable, EnumType, FloatRange, HasIO, \
    IntRange, Parameter, Property, Readable, nopoll, IDLE, WARN, ERROR
from frappy.errors import HardwareError
from frappy.states import HasStates, Retry, status_code
from frappy import datatypes

from .modbusTCP import ModbusIO, RegisterDef


# ==============================================================================
# CiA-402 device profile
# ==============================================================================

class StatusWord(IntFlag):
    """Bits of the CiA-402 statusword (object 0x6041)."""
    READY_TO_SWITCH_ON = 0x0001
    SWITCHED_ON = 0x0002
    OPERATION_ENABLED = 0x0004
    FAULT = 0x0008
    VOLTAGE_ENABLED = 0x0010
    QUICK_STOP = 0x0020
    SWITCH_ON_DISABLED = 0x0040
    WARNING = 0x0080
    REMOTE = 0x0200
    # bit 10: meaning is mode-dependent - 'target reached' in the profiled
    # modes (PPM/PVM/HMM), but RESERVED in CSP/CSV (firmware spec table 3-42).
    # Never test it in a cyclic synchronous mode: it may read 0 forever, or
    # read 1 while the axis is still turning.
    TARGET_REACHED = 0x0400
    INTERNAL_LIMIT = 0x0800
    # bit 12: meaning is mode-dependent - 'set-point acknowledged' in Profile
    # Position Mode (what we use it for), 'homing attained' in Homing mode,
    # 'drive follows the command value' in CSP/CSV. In CSV that is a useful
    # diagnostic but NOT a standstill test: it says the drive is tracking
    # *a* set value, not that the set value is zero.
    SETPOINT_ACK = 0x1000


class ControlWord(IntEnum):
    """CiA-402 controlword commands (object 0x6040, low byte).

    Operating-mode motion bits (PPM bit 4 'new set-point', bit 6 abs/rel,
    bit 8 'halt') are OR-ed onto ENABLE_OPERATION *after* the drive is
    enabled - they are not part of the state-machine sequence.

    The cyclic synchronous modes ignore ALL of those mode-specific bits
    ("Cyclic Synchronous Velocity Mode does not use mode-specific controlword
    bits", firmware spec 3.7.1), so in CSP/CSV only the state-machine commands
    below have any effect.
    """
    DISABLE_VOLTAGE = 0x0000        # -> switch on disabled  (coast)
    QUICK_STOP = 0x0002             # -> quick stop active
    SHUTDOWN = 0x0006               # -> ready to switch on
    SWITCH_ON = 0x0007              # -> switched on  (== 'disable operation')
    ENABLE_OPERATION = 0x000F       # -> operation enabled
    # enabled, holding (halt bit 8). Profiled modes only - CSP/CSV ignore it,
    # so writing this does NOT stop a cyclic axis. See _command_standstill.
    ENABLE_OPERATION_HALT = 0x010F
    FAULT_RESET = 0x0080            # rising edge of bit 7 clears a fault


class OperatingMode(IntEnum):
    """Modes of operation (object 0x6060)."""
    NO_MODE = 0            # what mode_display reads before anything is set
    PROFILE_POSITION = 1   # PPM
    PROFILE_VELOCITY = 3   # PVM
    HOMING = 6              # HMM
    CYCLIC_SYNC_POSITION = 8   # CSP
    CYCLIC_SYNC_VELOCITY = 9   # CSV


class DriveState(IntEnum):
    """States of the CiA-402 state machine.

    Integer valued so it can back a SECoP :class:`EnumType`; the human
    readable wording lives in :data:`DRIVE_STATE_TEXT`.
    """
    NOT_READY = 0
    SWITCH_ON_DISABLED = 1
    READY_TO_SWITCH_ON = 2
    SWITCHED_ON = 3
    OPERATION_ENABLED = 4
    QUICK_STOP_ACTIVE = 5
    FAULT_REACTION_ACTIVE = 6
    FAULT = 7


DRIVE_STATE_TEXT = {
    DriveState.NOT_READY: 'not ready to switch on',
    DriveState.SWITCH_ON_DISABLED: 'switch on disabled',
    DriveState.READY_TO_SWITCH_ON: 'ready to switch on',
    DriveState.SWITCHED_ON: 'switched on',
    DriveState.OPERATION_ENABLED: 'operation enabled',
    DriveState.QUICK_STOP_ACTIVE: 'quick stop active',
    DriveState.FAULT_REACTION_ACTIVE: 'fault reaction active',
    DriveState.FAULT: 'fault',
}

DRIVE_STATE_ENUM = EnumType('drivestate',
                            **{s.name.lower(): int(s) for s in DriveState})


class StatusControl:
    """Decodes CiA-402 statuswords and produces the controlwords to answer them.

    This class only knows the CiA-402 state machine itself - it has no opinion
    on *when* it is safe to send a recovery controlword (e.g. whether leaving
    quick stop automatically is acceptable). That policy lives on
    :class:`EPOS4Motion` (``resume_from_quickstop``), one level up, because it
    depends on how a given axis is wired, not on the protocol.
    """

    # (mask, pattern, secop status code, drive state), first match wins
    _STATE_TABLE = (
        (0x004F, 0x0000, WARN, DriveState.NOT_READY),
        (0x004F, 0x0040, WARN, DriveState.SWITCH_ON_DISABLED),
        (0x006F, 0x0021, IDLE, DriveState.READY_TO_SWITCH_ON),
        (0x006F, 0x0023, IDLE, DriveState.SWITCHED_ON),
        (0x006F, 0x0027, IDLE, DriveState.OPERATION_ENABLED),
        (0x006F, 0x0007, WARN, DriveState.QUICK_STOP_ACTIVE),
        (0x004F, 0x000F, ERROR, DriveState.FAULT_REACTION_ACTIVE),
        (0x004F, 0x0008, ERROR, DriveState.FAULT),
    )

    @staticmethod
    def get_state(sw: int):
        """Decode the CiA-402 state from a statusword.

        :return: ``(secop status code, DriveState)``
        """
        for mask, pattern, code, state in StatusControl._STATE_TABLE:
            if (sw & mask) == pattern:
                return code, state
        return WARN, DriveState.NOT_READY

    @staticmethod
    def is_fault(sw: int) -> bool:
        return bool(sw & StatusWord.FAULT)

    @staticmethod
    def is_enabled(sw: int) -> bool:
        _, state = StatusControl.get_state(sw)
        return state == DriveState.OPERATION_ENABLED

    # one controlword transition per statusword reading
    _ENABLE_STEP = {
        DriveState.FAULT: ControlWord.FAULT_RESET,
        DriveState.SWITCH_ON_DISABLED: ControlWord.SHUTDOWN,
        DriveState.READY_TO_SWITCH_ON: ControlWord.SWITCH_ON,
        DriveState.SWITCHED_ON: ControlWord.ENABLE_OPERATION,
        DriveState.QUICK_STOP_ACTIVE: ControlWord.ENABLE_OPERATION,  # resume
    }

    @staticmethod
    def next_controlword(sw: int, resume_from_quickstop=True):
        """Controlword to send next to progress toward 'operation enabled'.

        Returns None when already enabled, when the drive is still powering up
        (NOT_READY) or reacting to a fault (nothing to send until that settles),
        or when it is in quick stop and `resume_from_quickstop` is False - the
        caller has decided leaving quick stop is not this driver's decision to
        make (see :class:`EPOS4Motion`). Otherwise send the word, re-read the
        statusword, call again - one transition per call.
        """
        _, state = StatusControl.get_state(sw)
        if state in (DriveState.OPERATION_ENABLED, DriveState.NOT_READY,
                     DriveState.FAULT_REACTION_ACTIVE):
            return None
        if state == DriveState.QUICK_STOP_ACTIVE and not resume_from_quickstop:
            return None
        return StatusControl._ENABLE_STEP.get(state, ControlWord.SHUTDOWN)

    @staticmethod
    def fault_reset_word() -> int:
        return int(ControlWord.FAULT_RESET)

    @staticmethod
    def disable_word() -> int:
        return int(ControlWord.DISABLE_VOLTAGE)

    @staticmethod
    def quickstop_word() -> int:
        return int(ControlWord.QUICK_STOP)


# ==============================================================================
# Register map - addresses are RELATIVE to each drive's gateway window
# ==============================================================================

_U16 = dict(datatype=datatypes.UInt16,
            encode=ModbusIO.u16_to_registers,
            decode=ModbusIO.registers_to_u16)
_I32 = dict(datatype=datatypes.Int32, length=2,
            encode=ModbusIO.i32_to_registers,
            decode=ModbusIO.registers_to_i32)
_U8 = dict(datatype=datatypes.UInt8, length=1,
            encode=ModbusIO.u8_to_registers,
            decode=ModbusIO.registers_to_u8)

TAG_MAP = {
    # --- input registers (read) ---
    "read_status_word": RegisterDef(address=0, tag="Statusword", **_U16),
    "read_control_word": RegisterDef(address=1, tag="Controlword", **_U16),
    "read_error_code": RegisterDef(address=2, tag="Error_code", **_U16),
    "read_mode_of_operation": RegisterDef(
        address=3, tag="Modes_of_operation_display", **_U8),
    "read_position_value": RegisterDef(
        address=4, tag="Position_actual_value", **_I32),
    "read_position_demand": RegisterDef(
        address=6, tag="Position_demand_value", **_I32),
    "read_velocity_value": RegisterDef(
        address=8, tag="Velocity_actual_value_averaged", **_I32),
    "read_velocity_demand": RegisterDef(
        address=10, tag="Velocity_demand_value", **_I32),
    "read_current_value": RegisterDef(
        address=12, tag="Current_actual_value_averaged", **_I32),
    "read_torque_value": RegisterDef(
        address=14, tag="Torque_actual_value_averaged", **_U16),
    # --- holding registers (write) ---
    "write_control_word": RegisterDef(
        address=0, tag="Controlword", writable=True, **_U16),
    "write_mode_of_operation": RegisterDef(
        address=1, tag="Modes_of_operation", writable=True, **_U16),
    "write_target_position": RegisterDef(
        address=2, tag="Target_position", writable=True, **_I32),
    "write_target_velocity": RegisterDef(
        address=4, tag="Target_velocity", writable=True, **_I32),
    "write_target_voltage": RegisterDef(
        address=6, tag="Analog_output_general_purpose_A", writable=True, **_I32),
}


# ==============================================================================
# SECoP modules
# ==============================================================================

class EPOS4Base(HasIO, Readable):
    """Everything common to an EPOS4 drive reached through a Modbus gateway.

    Holds the register windowing, the single-transaction block read and the
    CiA-402 status decoding. On its own this is a read-only diagnostics module;
    subclass :class:`EPOS4Motion` to command the drive.
    """
    ioClass = ModbusIO

    # NB: not named read_offset/write_offset - frappy reserves the read_/write_
    # attribute prefixes for parameter accessors.
    input_offset = Property(
        "base address of this drive's input-register (read) window",
        datatype=IntRange(0, 0xFFFF), default=0)
    holding_offset = Property(
        "base address of this drive's holding-register (write) window",
        datatype=IntRange(0, 0xFFFF), default=0)
    counts_per_turn = Property(
        'encoder increments per revolution of the MOTOR/encoder shaft',
        datatype=FloatRange(), default=4096.0)
    gear_ratio = Property(
        'output-shaft turns per motor turn (>1 for a reduction gearbox); '
        'encoder counts per OUTPUT-shaft turn = counts_per_turn * gear_ratio',
        datatype=FloatRange(), default=1.0)
    velocity_scale = Property(
        'rpm per raw velocity unit reported by the drive',
        datatype=FloatRange(0), default=1.0)

    # the whole read side is one contiguous span, so a poll costs one
    # transaction regardless of how many parameters are exposed
    _READ_KEYS = tuple(k for k in TAG_MAP if k.startswith('read_'))
    _BLOCK_START = min(TAG_MAP[k].address for k in _READ_KEYS)
    _BLOCK_LEN = max(TAG_MAP[k].address + TAG_MAP[k].length
                     for k in _READ_KEYS) - _BLOCK_START

    value = Parameter('primary measured value - meaning depends on the motion '
                      'profile in use (see the concrete subclass)',
                      FloatRange(unit='rpm'))
    drive_state = Parameter('CiA-402 state of the drive', DRIVE_STATE_ENUM,
                            readonly=True, default=0, group='drive')
    error_code = Parameter('EPOS4 error code', IntRange(0, 0xFFFF),
                           readonly=True, default=0, group='drive')
    statusword = Parameter('raw CiA-402 statusword', IntRange(0, 0xFFFF),
                           readonly=True, default=0, group='drive',
                           visibility='expert')
    mode_display = Parameter('active mode of operation (see OperatingMode)',
                             IntRange(0, 11), readonly=True, default=0,
                             group='drive', visibility='expert')
    position = Parameter('shaft position, wrapped to one turn', FloatRange(unit='deg'),
                         readonly=True, default=0)
    velocity = Parameter('actual rotation speed', FloatRange(unit='rpm'),
                         readonly=True, default=0)
    velocity_demand = Parameter(
        'speed demanded by the profile generator - if this does not follow '
        'target, the ramp is not running (check mode, profile acceleration '
        'and max profile velocity)',
        FloatRange(unit='rpm'), readonly=True, default=0, group='drive')
    current = Parameter('averaged motor current', FloatRange(unit='mA'),
                        readonly=True, default=0, group='diagnostics',
                        visibility='expert')
    torque = Parameter('averaged torque, per mille of rated torque',
                       FloatRange(unit='permille'), readonly=True, default=0,
                       group='diagnostics', visibility='expert')

    _last_cw = None        # last controlword written, for readback checks
    _fields = None         # last decoded {tag key: raw value}
    _fields_ts = 0.0       # monotonic timestamp of that block
    _fields_max_age = 1.5  # only guards ad-hoc reads, never the poll path

    # -- unit conversion ---------------------------------------------------
    def _raw_to_deg(self, raw):
        return raw * 360.0 / (self.counts_per_turn * self.gear_ratio) % 360

    def _deg_to_raw(self, deg):
        """Inverse of :meth:`_raw_to_deg`, WITHOUT the wrap - a position target
        is an absolute encoder count, not a display value, so a target of
        e.g. 370 deg is a real, valid, more-than-one-turn destination."""
        return int(round(deg / 360.0 * self.counts_per_turn * self.gear_ratio))

    def _raw_to_rpm(self, raw):
        return raw * self.velocity_scale

    def _rpm_to_raw(self, rpm):
        return int(round(rpm / self.velocity_scale))

    # -- register access ---------------------------------------------------
    def _refresh_fields(self):
        """Fetch this drive's whole input window and decode every field once."""
        regs = self.io.fetch_block(self._BLOCK_START + self.input_offset,
                                   self._BLOCK_LEN)
        base = self._BLOCK_START
        self._fields = {
            key: TAG_MAP[key].decode(
                regs[TAG_MAP[key].address - base:
                     TAG_MAP[key].address - base + TAG_MAP[key].length])
            for key in self._READ_KEYS
        }
        self._fields_ts = time.monotonic()
        return self._fields

    def _field(self, key):
        """Decoded field for ad-hoc reads, refreshing only when stale.

        State functions must use :meth:`_drv` instead - ``doPoll`` has already
        refreshed the block for them in the same cycle.
        """
        if (self._fields is None
                or time.monotonic() - self._fields_ts > self._fields_max_age):
            self._refresh_fields()
        return self._fields[key]

    def _drv(self, key):
        """Decoded field from the block ``doPoll`` fetched this cycle."""
        return self._fields[key]

    def _write(self, key, value):
        return self.io.set(TAG_MAP[key], value, offset=self.holding_offset)

    def _write_controlword(self, cw):
        """Write the controlword, remembering it so a readback can be checked."""
        self._last_cw = int(cw)
        return self._write('write_control_word', int(cw))

    # -- polling -----------------------------------------------------------
    def doPoll(self):
        """Refresh every published parameter from one Modbus transaction."""
        fields = self._refresh_fields()
        sw = fields['read_status_word']
        _, state = StatusControl.get_state(sw)
        self.statusword = sw
        self.drive_state = int(state)
        self.error_code = fields['read_error_code']
        self.mode_display = fields['read_mode_of_operation']
        self.position = self._raw_to_deg(fields['read_position_value'])
        self.velocity = self._raw_to_rpm(fields['read_velocity_value'])
        self.velocity_demand = self._raw_to_rpm(fields['read_velocity_demand'])
        self.current = float(fields['read_current_value'])
        self.torque = fields['read_torque_value'] / 1000.0
        self._update_value()
        self.read_status()

    def _update_value(self):
        """Which measurement ``value`` mirrors. Default: rotation speed.

        Overridden by :class:`EPOS4Position` (position instead).
        """
        self.value = self.velocity

    # -- on-demand readers (the poll path is doPoll, so none of these poll) --
    @nopoll
    def read_value(self):
        """Default: value mirrors rotation speed - overridden by
        EPOS4Position, whose value mirrors position instead."""
        return self.read_velocity()

    @nopoll
    def read_velocity(self):
        return self._raw_to_rpm(self._field('read_velocity_value'))

    @nopoll
    def read_position(self):
        return self._raw_to_deg(self._field('read_position_value'))

    @nopoll
    def read_statusword(self):
        return self._field('read_status_word')

    @nopoll
    def read_error_code(self):
        return self._field('read_error_code')

    @nopoll
    def read_velocity_demand(self):
        return self._raw_to_rpm(self._field('read_velocity_demand'))

    @nopoll
    def read_mode_display(self):
        return self._field('read_mode_of_operation') & 0xFF

    @nopoll
    def read_current(self):
        return float(self._field('read_current_value'))

    @nopoll
    def read_torque(self):
        return self._field('read_torque_value') / 1000.0

    @nopoll
    def read_drive_state(self):
        _, state = StatusControl.get_state(self._field('read_status_word'))
        return int(state)

    @nopoll
    def read_status(self):
        code, state = StatusControl.get_state(self._field('read_status_word'))
        if code >= ERROR:
            return code, (f'{DRIVE_STATE_TEXT[state]} '
                          f'(error 0x{self._field("read_error_code"):04X})')
        return code, DRIVE_STATE_TEXT[state]


class EPOS4Motion(HasStates, Drivable, EPOS4Base):
    """Shared CiA-402 enable/disable/fault handling for a driven EPOS4 axis.

    Not used directly - subclass and set ``OPERATING_MODE`` plus a ``running``
    state function, and - if the mode does not respond to the CiA-402 halt bit
    - a ``_command_standstill``. See :class:`EPOS4Velocity` and
    :class:`EPOS4Position`.

    ``write_target`` runs a non-blocking state machine: :meth:`enabling` walks
    the drive to 'operation enabled' in ``OPERATING_MODE``, one CiA-402
    controlword transition per poll cycle, then hands off to the subclass's
    ``running`` state. ``status`` is BUSY throughout, IDLE once ``running``
    calls :meth:`~frappy.states.HasStates.final_status`, and WARN whenever the
    drive itself is not actually enabled - regardless of what the state
    machine last finished at (see :meth:`read_status`).
    """
    #: (int) object 0x6060 value this class puts the drive into. Set by
    #: subclasses; EPOS4Motion itself is not a complete, instantiable module.
    OPERATING_MODE = None

    #: Whether the enable sequence is allowed to send ENABLE_OPERATION to move
    #: the drive out of Quick Stop Active on its own. Default False: quick
    #: stop is treated as an externally-imposed condition that this driver must
    #: never talk the drive out of - only hardware/an operator gets to end it.
    #: Set True in the module's config only for an axis where automatic
    #: recovery from quick stop is actually wanted.
    #:
    #: NB this guards ONLY Quick Stop Active. An interlock wired to a digital
    #: input configured as 'Drive enable' (function 27) rather than 'Quick
    #: stop' (28) leaves the drive in Switch On Disabled instead, which the
    #: ladder in StatusControl._ENABLE_STEP will happily walk back up.
    resume_from_quickstop = Property(
        'allow the enable sequence to leave Quick Stop Active by sending '
        'Enable Operation. Leave False for any axis whose interlock is wired '
        'to a digital input configured as Quick stop - see the class docstring.',
        datatype=datatypes.BoolType(), default=False)

    #: Whether stop() switches the power stage off once the axis is at rest.
    #: Default False: the drive stays enabled and actively holds zero speed.
    #: True leaves the shaft free to turn by hand - useful for sample access.
    #: Either way the next target write re-enables the drive on its own.
    disable_after_stop = Property(
        'after stop() has brought the axis to a standstill, send Disable '
        'Operation (controlword 0x0007) to switch off the power stage, leaving '
        'the shaft free to turn by hand. Default False: the drive stays '
        'enabled and actively holds zero speed.',
        datatype=datatypes.BoolType(), default=False)

    enable_timeout = Parameter('timeout for the CiA-402 enable sequence '
                               '(irrelevant while waiting out quick stop - '
                               'see resume_from_quickstop)',
                               FloatRange(0, unit='s'), default=3.0,
                               readonly=False, group='tuning')
    stop_timeout = Parameter('timeout for coming to a standstill after stop(). '
                             'This is only a diagnostic budget - what actually '
                             'sets the deceleration is the drive: the slowdown '
                             'ramp (0x6084) for a halt, or Max acceleration '
                             '(0x60C5) where the set-point comes from an analog '
                             'input, as in CSV',
                             FloatRange(0, unit='s'), default=10.0,
                             readonly=False, group='tuning')
    stop_speed_tolerance = Parameter(
        'speed considered "stopped" for the purposes of stop()',
        FloatRange(0, unit='rpm'), default=2.0, readonly=False, group='tuning')

    _blocking_fault = None
    _idle_disabled = False  # power stage switched off deliberately by stop()

    # -- driving -----------------------------------------------------------
    def write_target(self, value):
        if self._blocking_fault:
            raise HardwareError(
                f'{self._blocking_fault} - run reset_fault first')
        # a new move supersedes any deliberate power-down done by stop(), so
        # 'not enabled' goes back to meaning something is wrong (read_status)
        self._idle_disabled = False
        self.start_machine(self.enabling, target=value)
        return value

    @status_code(BUSY, 'enabling drive')
    def enabling(self, sm):
        if self._fields is None:
            return Retry  # no block read yet
        if sm.init:
            self._write('write_mode_of_operation', int(self.OPERATING_MODE))
            sm.deadline = sm.now + self.enable_timeout
        sw = self._drv('read_status_word')
        _, state = StatusControl.get_state(sw)

        if state == DriveState.QUICK_STOP_ACTIVE and not self.resume_from_quickstop:
            # Waiting out an externally-imposed quick stop is not a timeout
            # condition - keep pushing the deadline forward so this can sit
            # here indefinitely without ever raising, and send nothing.
            # Whatever cleared the interlock/quick-stop will move the drive to
            # a different state on its own; normal enabling resumes from there
            # next cycle, having sent zero controlwords in the meantime.
            sm.deadline = sm.now + self.enable_timeout
            return Retry

        if StatusControl.is_enabled(sw):
            mode = self._drv('read_mode_of_operation') & 0xFF
            if mode != self.OPERATING_MODE:
                # The mode write at sm.init may not have landed (or something
                # else changed it) - re-assert rather than assume.
                self._write('write_mode_of_operation', int(self.OPERATING_MODE))
                if sm.now > sm.deadline:
                    raise HardwareError(
                        f'drive is enabled but stayed in mode {mode} instead '
                        f'of {int(self.OPERATING_MODE)} - target will be '
                        f'ignored. Check that write_mode_of_operation is '
                        f'mapped correctly.')
                return Retry
            return self.running
        cw = StatusControl.next_controlword(
            sw, resume_from_quickstop=self.resume_from_quickstop)
        if cw is not None:
            self._write_controlword(cw)
        if sm.now > sm.deadline:
            raise HardwareError(
                f'drive did not enable, stuck in {DRIVE_STATE_TEXT[state]!r} '
                f'after {self.enable_timeout:g} s. {self._enable_diagnosis(sw)}')
        return Retry

    def running(self, sm):
        """Entered once the drive is enabled and in ``OPERATING_MODE``.

        Must be provided by a concrete subclass.
        """
        raise NotImplementedError(
            f'{type(self).__name__} must implement running() - subclass '
            f'EPOS4Velocity or EPOS4Position, not EPOS4Motion directly')

    def _enable_diagnosis(self, sw):
        """Say which half of the chain is at fault when enabling times out.

        The controlword is mapped for reading as well, so comparing what we
        wrote against what the drive reports separates 'the write never
        arrived' from 'the drive refuses to move'.
        """
        err = self._drv('read_error_code')
        mode = self._drv('read_mode_of_operation') & 0xFF
        try:
            readback = self._drv('read_control_word')
        except KeyError:
            readback = None
        detail = (f'statusword 0x{sw:04X}, error 0x{err:04X}, mode {mode}, '
                  f'last controlword written '
                  f'0x{(self._last_cw or 0):04X}')
        if readback is None:
            return detail + ' (no controlword readback mapped)'
        detail += f', controlword read back 0x{readback:04X}'
        if self._last_cw is not None and readback != self._last_cw:
            return (detail + ' -> READBACK MISMATCH: the write is not reaching '
                    'the drive. Check holding_offset and the gateway write '
                    'mapping, and try use_fc06 on the io module.')
        return (detail + ' -> the controlword arrived but the drive will not '
                'leave this state. Check STO/enable wiring, motor supply '
                'voltage, and the error code above.')

    # -- stopping ----------------------------------------------------------
    def _command_standstill(self):
        """Tell the drive to come to a standstill, however this profile does it.

        Must not send a controlword that could resume motion, and must not
        disable the power stage - :meth:`stopping` does that separately, and
        only once the axis is already at rest.

        Base implementation: the CiA-402 halt bit, which is what Profile
        Position Mode uses. That is only safe because
        :meth:`EPOS4Position.running` writes a plain ENABLE_OPERATION at
        ``sm.init``, clearing bit 8 again on the next move - a subclass whose
        ``running`` does not do that MUST override this, or the axis will stay
        halted forever. :class:`EPOS4Velocity` overrides it for a different
        reason: the cyclic synchronous modes ignore every mode-specific
        controlword bit, the halt bit included (firmware spec 3.7.1).
        """
        self._write_controlword(ControlWord.ENABLE_OPERATION_HALT)

    @status_code(BUSY, 'stopping')
    def stopping(self, sm):
        """Wait out the standstill that :meth:`stop` has already commanded.

        The test is the MEASURED speed, and only that. Statusword bit 10
        ('target reached') is deliberately not consulted: its meaning is
        mode-specific and it is reserved in the cyclic synchronous modes
        (firmware spec table 3-42), so it cannot mean the same thing for every
        subclass of this class. Actual shaft speed can.
        """
        if self._fields is None:
            return Retry  # no block read yet
        if sm.init:
            sm.deadline = sm.now + self.stop_timeout
        sw = self._drv('read_status_word')
        speed = self._raw_to_rpm(self._drv('read_velocity_value'))
        if abs(speed) <= self.stop_speed_tolerance:
            # Only disable a drive that is actually ours to disable. If it has
            # already left 'operation enabled', something else stopped it (an
            # interlock, most likely) and claiming credit here would mask that
            # as a tidy 'power stage off' in read_status.
            if self.disable_after_stop and StatusControl.is_enabled(sw):
                self._write_controlword(ControlWord.SWITCH_ON)
                self._idle_disabled = True
            return self.final_status(IDLE, 'stopped')
        if sm.now > sm.deadline:
            return self.final_status(
                WARN, f'still turning at {speed:g} rpm {self.stop_timeout:g} s '
                f'after stop was commanded (statusword 0x{sw:04X})')
        return Retry

    @Command
    def stop(self):
        """Bring the axis to a standstill, leaving the power stage enabled
        unless ``disable_after_stop`` is set.

        How a standstill is commanded is mode-specific (see
        :meth:`_command_standstill`); that it has been reached is judged from
        the measured speed alone.
        """
        # Immediately, in the caller's thread. Deferring this to the state
        # machine would mean up to a pollinterval of continued rotation after
        # the operator asked for a stop. Same reasoning as
        # EPOS4Velocity.write_target, which also writes its setpoint straight
        # away rather than from a state function.
        self._command_standstill()
        # Only announced once that write actually reached the drive: if it
        # raises, target keeps reading the speed that is in fact still
        # commanded, rather than a reassuring zero.
        self.target = 0
        self.start_machine(self.stopping)

    @Command(IntRange(0, 0xFFFF), result=IntRange(0, 0xFFFF),
             visibility='expert')
    def raw_controlword(self, value):
        """Write a raw controlword and return the drive's readback.

        For bench-testing the gateway's write mapping without the state
        machine: step 0x06 -> 0x07 -> 0x0F by hand and watch `statusword`
        move 0x0040 -> 0x0021 -> 0x0023 -> 0x0027. If the returned readback
        does not match what you wrote, the write is not reaching the drive.
        """
        self._write_controlword(value)
        self._refresh_fields()
        self.read_status()
        return self._drv('read_control_word')

    @Command
    def reset_fault(self):
        """Acknowledge a drive fault (rising edge of controlword bit 7)."""
        self._write_controlword(StatusControl.disable_word())
        self._write_controlword(StatusControl.fault_reset_word())
        self._blocking_fault = None
        self._refresh_fields()
        self.read_status()

    def on_error(self, sm):
        reason = repr(sm.cleanup_reason)
        self._blocking_fault = reason
        try:
            self._write_controlword(StatusControl.disable_word())
        except Exception as e:
            self.log.warning('could not disable drive after error: %r', e)
        return super().on_error(sm)

    # -- status ------------------------------------------------------------
    def read_status(self):
        """Merge CiA-402 drive faults with the state machine's status.

        This runs every poll regardless of whether the state machine is
        actively doing anything, so it is what makes the driver reflect a
        drive that has dropped out of 'operation enabled' (e.g. an interlock
        engaging) as WARN immediately, and automatically go back to reporting
        the machine's last IDLE status once the drive is enabled again - with
        no further action from the state machine required either way.
        """
        sw = self._field('read_status_word')
        code, state = StatusControl.get_state(sw)
        if code >= ERROR:
            self._blocking_fault = (
                f'{DRIVE_STATE_TEXT[state]} '
                f'(error 0x{self._field("read_error_code"):04X})')
            return code, self._blocking_fault
        if self._blocking_fault:
            return ERROR, f'{self._blocking_fault} - run reset_fault'
        if state == DriveState.QUICK_STOP_ACTIVE and not self.resume_from_quickstop:
            # Show this immediately and unconditionally, even while the state
            # machine is busy in enabling() waiting it out (e.g. a client
            # just wrote a new target): "BUSY enabling drive" would otherwise
            # be shown for as long as the interlock is engaged, which reads as
            # "something is actively happening" when the true, and important,
            # state is "waiting for something outside Frappy's control".
            return WARN, DRIVE_STATE_TEXT[state]
        machine = super().read_status()  # HasStates -> state machine status
        if machine[0] == IDLE and state != DriveState.OPERATION_ENABLED:
            if self._idle_disabled and state == DriveState.SWITCHED_ON:
                # stop() switched the power stage off on purpose
                # (disable_after_stop) - that is not something to warn about.
                # Matching SWITCHED_ON only is deliberate: a drive disabled by
                # an interlock lands elsewhere and still reports WARN below.
                return IDLE, 'stopped, power stage off'
            # idle, but the drive is not actually under power
            return WARN, DRIVE_STATE_TEXT[state]
        return machine


class EPOS4Velocity(EPOS4Motion):
    """Continuous rotation commanded through an analog output voltage.

    Why not simply write the CANopen velocity object (0x60FF) in Profile
    Velocity Mode, as this driver originally did? PVM's profile generator only
    picks up a new set-point when a controlword arrives (a rising edge on the
    'new set-point' handshake, or - as this driver used to do it - a
    re-assertion of Enable Operation); the write to 0x60FF alone does nothing.
    That is fine on its own, but this axis also has a safety interlock, and
    reacting to it by re-sending Enable Operation is exactly the kind of
    "Frappy sends a controlword to restart motion" that must not happen here:
    the interlock has to be able to stop AND release the drive purely through
    its own wiring, with no software step in between.

    So instead the drive runs in Cyclic Synchronous Velocity mode (0x6060 = 9)
    and ``target`` is not written to 0x60FF at all - it is converted to a
    voltage and written to one of the drive's analog outputs (object 0x3240,
    "Analog output general purpose A" in this register map). That output is
    wired back into the same EPOS4's own analog input, which is configured as
    "Set value velocity" (0x3161 = 9) so that CSV takes its cyclic set-point
    from the analog value rather than from 0x60FF, with the scaling set by
    0x3171 ("Analog input velocity set value properties").

    Because the analog output register is a plain, always-on value with no
    "apply this" handshake, writing ``target`` is a one-shot register write,
    done immediately and unconditionally, decoupled from the CiA-402 enable
    state entirely - it happens whether or not the drive currently reports
    'operation enabled'. The state machine's job is purely to get the drive
    into CSV mode and confirm it is enabled; it never needs to, and never
    does, re-touch the voltage once written.

    **Stopping.** The halt bit this class inherits from :class:`EPOS4Motion` is
    inoperative here - "Cyclic Synchronous Velocity Mode does not use
    mode-specific controlword bits" (firmware spec 3.7.1) - so
    :meth:`_command_standstill` is overridden to lower the analog output to
    ``zero_speed_voltage`` instead. That is the only honest stop on this axis:
    the analog voltage is what the drive resumes from, so a stop that leaves it
    high would spin the axis straight back up the moment anything returned the
    drive to 'operation enabled'. It also means a stop writes no controlword at
    all, which is the same principle the class is built around. The
    deceleration is set by the drive's Max acceleration (0x60C5), which limits
    how fast the analog set value may slew - not by ``stop_timeout``, and not
    by the profile deceleration used by a halt.

    Quick stop would also work in CSV (it is a state-machine transition rather
    than a mode-specific bit), but it is not used: it cannot replace zeroing
    the voltage for the reason above, and Quick stop option code 0x605A is
    fixed at 6 on the EPOS4 - "decelerate with quick stop ramp and *stay in*
    Quick stop active" - so it would latch the axis in a state this driver
    refuses to leave. Raise 0x60C5 in EPOS Studio if the ramp-down is too slow.

    **The interlock.** On the tumbler it gates the drive's 'Drive enable'
    digital input (0x3142 function 27), not Quick stop (28), so engaging it
    drops the drive out of 'operation enabled' rather than into Quick Stop
    Active. Two consequences worth knowing:

    - ``resume_from_quickstop`` (inherited, default False) does NOT guard this
      axis, because the ladder in ``StatusControl._ENABLE_STEP`` will walk a
      drive back up out of Switch On Disabled. Leave it False regardless - it
      costs nothing and is correct for any axis that is wired the other way.
    - :meth:`running` already handles the drive leaving 'operation enabled'
      without chasing it: the correct voltage stays asserted, the state machine
      sits still, and :meth:`read_status` reports WARN until the drive comes
      back. No controlword is sent at any point, which is what lets the
      interlock stop and release the axis entirely on its own.
    """
    OPERATING_MODE = OperatingMode.CYCLIC_SYNC_VELOCITY

    # Bounds match the zero_speed_voltage/rpm_per_volt/output_voltage_*
    # defaults just below: (2.3-1.0)*1000 = 1300 rpm. This is a genuine
    # SECoP-level range check (a client sending outside it gets a clean
    # RangeError) rather than the voltage-clamp in _rpm_to_voltage, which is
    # only a defence against cfg drift once these numbers no longer agree - if
    # you override the calibration Properties for a different axis, update
    # this range to match, or a client can request a target that gets
    # silently clamped to a different voltage than the reported rpm implies.
    target = Parameter('commanded rotation speed', FloatRange(0, 1300, unit='rpm'),
                       readonly=False, default=0)
    value = Parameter('actual rotation speed', FloatRange(unit='rpm'))

    zero_speed_voltage = Property(
        'analog output voltage that commands 0 rpm', FloatRange(), default=1.0)
    rpm_per_volt = Property(
        "slope of the external speed scaling: rpm change per volt above "
        "zero_speed_voltage. Default 1000.0 matches '1 V = 0 rpm, 2.3 V = "
        "1300 rpm' -> (1300 - 0) / (2.3 - 1.0) = 1000",
        FloatRange(), default=1000.0)
    output_voltage_min = Property(
        'hard safety clamp: lowest voltage ever written, regardless of target. '
        'Defaults to exactly the characterised range (1.0-2.3 V); widen only '
        'after confirming the scaling stays linear beyond it',
        FloatRange(), default=1.0)
    output_voltage_max = Property(
        'hard safety clamp: highest voltage ever written, regardless of target',
        FloatRange(), default=2.3)

    speed_tolerance = Parameter(
        'speed deviation from target still counted as on target',
        FloatRange(0, unit='rpm'), default=70, readonly=False, group='tuning')
    settle_timeout = Parameter(
        'how long to wait for the measured speed to reach target before '
        'reporting it as a (non-fatal) diagnostic mismatch',
        FloatRange(0, unit='s'), default=20.0, readonly=False, group='tuning')

    output_voltage = Parameter(
        'voltage last written to the analog output (commanded, not read back '
        '- this register has no readback mapped)',
        FloatRange(unit='V'), readonly=True, default=0, group='drive')

    _MILLIVOLTS_PER_VOLT = 1000  # unit fact, not a calibration - not a Property

    def _rpm_to_voltage(self, rpm):
        vout = self.zero_speed_voltage + rpm / self.rpm_per_volt
        clamped = min(max(vout, self.output_voltage_min), self.output_voltage_max)
        if clamped != vout:
            self.log.warning(
                '%g rpm needs %.3f V, clamped to %.3f V by output_voltage_min/max',
                rpm, vout, clamped)
        return clamped

    def _voltage_to_raw(self, volts):
        return int(round(volts * self._MILLIVOLTS_PER_VOLT))

    def read_output_voltage(self):
        return self.output_voltage

    def _write_speed_reference(self, rpm):
        """Command ``rpm`` on the analog output and publish what was written.

        In CSV the set-point comes from the analog input, so writing the analog
        output IS commanding a speed. This is the single place that happens -
        both :meth:`write_target` and the stop path go through here, so
        ``target``, ``output_voltage`` and the register can never disagree
        about what this axis was last told.

        Note this deliberately goes through :meth:`_rpm_to_voltage` even for
        zero, rather than using ``zero_speed_voltage`` directly, so that a cfg
        whose voltage clamp no longer agrees with its calibration gets the
        usual warning on the stop path too instead of silently commanding a
        non-zero speed.
        """
        volts = self._rpm_to_voltage(rpm)
        self._write('write_target_voltage', self._voltage_to_raw(volts))
        # Belt and braces. The firmware spec is not explicit about whether the
        # analog 'Set value velocity' function replaces 0x60FF or is summed
        # with it (it does force Velocity offset 0x60B1 to zero - spec
        # 6.2.80.1). A stale non-zero 0x60FF - left by this driver's Profile
        # Velocity ancestor, or by a bench session in EPOS Studio - would be a
        # speed contribution the analog output cannot cancel, so stop() would
        # not stop. Asserting zero can only ever remove that term: the axis is
        # known to follow the analog voltage as it is.
        self._write('write_target_velocity', 0)
        self.output_voltage = volts
        return volts

    def write_target(self, value):
        # Unconditional and immediate: correct regardless of whether the drive
        # is currently enabled, and is never repeated as a side effect of
        # anything else (in particular, never as part of recovering from
        # quick stop - see the class docstring).
        self._write_speed_reference(value)
        return super().write_target(value)  # fault-check + start the state machine

    def _command_standstill(self):
        """CSV ignores the halt bit (firmware spec 3.7.1), so a stop here is
        simply a zero-speed command on the same analog output that every other
        speed goes through - no controlword involved."""
        self._write_speed_reference(0)

    @status_code(BUSY, 'confirming speed')
    def running(self, sm):
        """The voltage is already on the output by the time this runs (see
        write_target) - this only confirms the drive is actually turning at
        the expected speed, for diagnostics, and reports (without chasing)
        when it is not because the drive is not enabled."""
        if sm.init:
            sm.deadline = sm.now + self.settle_timeout
        sw = self._drv('read_status_word')
        if StatusControl.is_fault(sw):
            raise HardwareError(
                f'drive faulted while running '
                f'(error 0x{self._drv("read_error_code"):04X})')
        if not StatusControl.is_enabled(sw):
            # Left 'operation enabled' on its own (interlock, most likely).
            # The correct voltage is already asserted; there is nothing for
            # Frappy to do but say so. read_status() overrides this to WARN
            # with the live drive state on every poll for as long as this
            # lasts, and reverts automatically once the drive re-enables -
            # with the state machine sitting exactly here, untouched.
            return self.final_status(IDLE, f'holding {sm.target:g} rpm setpoint')
        speed = self._raw_to_rpm(self._drv('read_velocity_value'))
        if abs(speed - sm.target) <= self.speed_tolerance:
            if sm.target:
                return self.final_status(IDLE, f'rotating at {sm.target:g} rpm')
            return self.final_status(IDLE, 'stopped')
        if sm.now > sm.deadline:
            # Not fatal: the voltage is correct, so this is a downstream
            # scaling/wiring/mechanical question, not something to retry.
            return self.final_status(
                WARN, f'commanded {sm.target:g} rpm ({self.output_voltage:.3f} V) '
                f'but measured {speed:g} rpm after {self.settle_timeout:g} s - '
                f'check rpm_per_volt/zero_speed_voltage and the wiring')
        return Retry


class EPOS4Position(EPOS4Motion):
    """Point-to-point moves in Profile Position Mode (PPM, 0x6060 = 1).

    ``target`` degrees are converted through the same encoder-count scaling as
    the read-only ``position`` (see ``counts_per_turn`` / ``gear_ratio``), but
    UNWRAPPED - i.e. a target of 370 deg is a real destination more than one
    turn away, not equivalent to 10 deg. ``position`` itself still displays
    wrapped to one turn (0-360), so after a move to 370 deg, ``position`` will
    read 10 while ``target`` still reads 370. If this axis has a limited
    range of travel rather than tumbling freely, that is probably not the
    convention you want - say so and this can be changed to work in signed,
    unwrapped degrees throughout instead.

    Not currently exercised against hardware (unlike EPOS4Velocity) - the
    CiA-402 sequence (rising edge of controlword bit 4 latches a new set-point,
    bit 12 acknowledges it, bit 10 confirms arrival) is implemented per spec,
    but treat the timeouts/tolerances as starting points.

    Profile Position Mode - not Cyclic Synchronous Position - is used
    deliberately: CSP expects a fresh target every SYNC/interpolation cycle
    (typically millisecond-scale) or it faults on a following-error timeout,
    which a poll-driven Modbus/SDO gateway like this one cannot sustain. PPM's
    single-set-point-then-wait model is exactly this driver's shape.
    """
    OPERATING_MODE = OperatingMode.PROFILE_POSITION

    target = Parameter('target shaft position (unwrapped, see class docs)',
                       FloatRange(unit='deg'), readonly=False, default=0)
    value = Parameter('current shaft position, wrapped to one turn',
                      FloatRange(unit='deg'))

    profile_velocity = Parameter(
        'speed used while moving to a new target',
        FloatRange(0, unit='rpm'), default=100, readonly=False, group='tuning')
    position_tolerance = Parameter(
        'position deviation from target still counted as arrived',
        FloatRange(0, unit='deg'), default=1.0, readonly=False, group='tuning')
    move_timeout = Parameter(
        'timeout for a move to complete',
        FloatRange(0, unit='s'), default=30.0, readonly=False, group='tuning')
    ack_timeout = Parameter(
        'timeout for the drive to acknowledge a new set-point (statusword '
        'bit 12) - a short, separate budget from move_timeout, since an '
        'unacknowledged set-point will never move at all',
        FloatRange(0, unit='s'), default=2.0, readonly=False, group='tuning')

    def _update_value(self):
        self.value = self.position

    @nopoll
    def read_value(self):
        return self.read_position()

    @status_code(BUSY, 'moving')
    def running(self, sm):
        if sm.init:
            self._write('write_target_position', self._deg_to_raw(sm.target))
            self._write('write_target_velocity',
                        self._rpm_to_raw(self.profile_velocity))
            # PPM handshake: a RISING edge of controlword bit 4 latches the
            # new set-point. enabling() leaves the controlword at plain
            # ENABLE_OPERATION (bit 4 low), so raising it here is a real edge.
            self._write_controlword(int(ControlWord.ENABLE_OPERATION))
            self._write_controlword(int(ControlWord.ENABLE_OPERATION) | (1 << 4))
            sm.ack_deadline = sm.now + self.ack_timeout
            sm.move_deadline = sm.now + self.move_timeout
            sm.acked = False
            # the statusword in hand predates every write above
            return Retry
        sw = self._drv('read_status_word')
        if StatusControl.is_fault(sw):
            raise HardwareError(
                f'drive faulted while moving '
                f'(error 0x{self._drv("read_error_code"):04X})')
        if not StatusControl.is_enabled(sw):
            _, state = StatusControl.get_state(sw)
            return self.final_status(
                WARN, f'{DRIVE_STATE_TEXT[state]} - move interrupted')
        if not sm.acked:
            if sw & StatusWord.SETPOINT_ACK:
                sm.acked = True
                # drop bit 4 back to low so the NEXT move gets a fresh edge
                self._write_controlword(int(ControlWord.ENABLE_OPERATION))
                return Retry
            if sm.now > sm.ack_deadline:
                raise HardwareError(
                    'drive did not acknowledge the new set-point (statusword '
                    f'bit 12 never set, 0x{sw:04X}) - check '
                    'write_target_position/write_control_word mapping')
            return Retry
        if sw & StatusWord.TARGET_REACHED:
            return self.final_status(IDLE, f'at {sm.target:g} deg')
        if sm.now > sm.move_deadline:
            here = self._raw_to_deg(self._drv('read_position_value'))
            raise HardwareError(
                f'did not reach {sm.target:g} deg within {self.move_timeout:g} s '
                f'(at {here:g} deg)')
        return Retry
