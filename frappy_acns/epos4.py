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

Layering: :mod:`frappy_acns.modbusTCP` is the transport, this module is the
device family, and instrument-specific modules (e.g.
:mod:`frappy_acns.tumbler`) subclass :class:`EPOS4Motion`.
"""

import time
from enum import IntEnum, IntFlag

from frappy.core import Command, Drivable, EnumType, FloatRange, HasIO, \
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
    TARGET_REACHED = 0x0400
    INTERNAL_LIMIT = 0x0800


class ControlWord(IntEnum):
    """CiA-402 controlword commands (object 0x6040, low byte).

    Operating-mode motion bits (PPM bit 4 'new set-point', bit 6 abs/rel,
    bit 8 'halt') are OR-ed onto ENABLE_OPERATION *after* the drive is
    enabled - they are not part of the state-machine sequence.
    """
    DISABLE_VOLTAGE = 0x0000        # -> switch on disabled  (coast)
    QUICK_STOP = 0x0002             # -> quick stop active
    SHUTDOWN = 0x0006               # -> ready to switch on
    SWITCH_ON = 0x0007              # -> switched on  (== 'disable operation')
    ENABLE_OPERATION = 0x000F       # -> operation enabled
    ENABLE_OPERATION_HALT = 0x010F  # enabled, holding (halt bit 8)
    FAULT_RESET = 0x0080            # rising edge of bit 7 clears a fault


class OperatingMode(IntEnum):
    """Modes of operation (object 0x6060)."""
    PROFILE_POSITION = 1   # PPM
    PROFILE_VELOCITY = 3   # PVM
    HOMING = 6             # HMM


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
    """Decodes CiA-402 statuswords and produces the controlwords to answer them."""

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
    def next_controlword(sw: int):
        """Controlword to send next to progress toward 'operation enabled'.

        Returns None when already enabled, or while the drive is still
        powering up / decelerating on a fault and we should just wait. Send
        the word, re-read the statusword, call again - one transition per call.
        """
        _, state = StatusControl.get_state(sw)
        if state in (DriveState.OPERATION_ENABLED, DriveState.NOT_READY,
                     DriveState.FAULT_REACTION_ACTIVE):
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
}


# ==============================================================================
# SECoP modules
# ==============================================================================

class EPOS4Base(HasIO, Readable):
    """Everything common to an EPOS4 drive reached through a Modbus gateway.

    Holds the register windowing, the single-transaction block read and the
    CiA-402 status decoding. Subclass and add motion behaviour.
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
        'encoder increments per revolution of the output shaft',
        datatype=FloatRange(), default=4096.0)
    velocity_scale = Property(
        'rpm per raw velocity unit reported by the drive',
        datatype=FloatRange(0), default=1.0)

    # the whole read side is one contiguous span, so a poll costs one
    # transaction regardless of how many parameters are exposed
    _READ_KEYS = tuple(k for k in TAG_MAP if k.startswith('read_'))
    _BLOCK_START = min(TAG_MAP[k].address for k in _READ_KEYS)
    _BLOCK_LEN = max(TAG_MAP[k].address + TAG_MAP[k].length
                     for k in _READ_KEYS) - _BLOCK_START

    value = Parameter('actual rotation speed', FloatRange(unit='rpm'))
    drive_state = Parameter('CiA-402 state of the drive', DRIVE_STATE_ENUM,
                            readonly=True, default=0, group='drive')
    error_code = Parameter('EPOS4 error code', IntRange(0, 0xFFFF),
                           readonly=True, default=0, group='drive')
    statusword = Parameter('raw CiA-402 statusword', IntRange(0, 0xFFFF),
                           readonly=True, default=0, group='drive',
                           visibility='expert')
    mode_display = Parameter('active mode of operation', IntRange(0, 11),
                             readonly=True, default=0, group='drive',
                             visibility='expert')
    position = Parameter('shaft position', FloatRange(unit='deg'),
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
        return raw * 360.0 / (self.counts_per_turn * 130) % 360

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
        self.mode_display = fields['read_mode_of_operation'] # & 0xFF
        self.position = self._raw_to_deg(fields['read_position_value'])
        self.velocity = self._raw_to_rpm(fields['read_velocity_value'])
        self.velocity_demand = self._raw_to_rpm(fields['read_velocity_demand'])
        self.current = float(fields['read_current_value'])
        self.torque = fields['read_torque_value'] / 1000.0
        self.value = self.velocity
        self.read_status()

    # -- on-demand readers (the poll path is doPoll, so none of these poll) --
    @nopoll
    def read_value(self):
        return self._raw_to_rpm(self._field('read_velocity_value'))

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
    """An EPOS4 running in Profile Velocity Mode: continuous rotation.

    ``target`` is the commanded shaft speed. Writing it runs a non-blocking
    CiA-402 state machine that enables the power stage one transition per poll
    cycle and then ramps to speed; ``status`` is BUSY until the drive reports
    target reached.
    """
    target = Parameter('commanded rotation speed', FloatRange(unit='rpm'),
                       readonly=False, default=0)

    enable_timeout = Parameter('timeout for the CiA-402 enable sequence',
                               FloatRange(0, unit='s'), default=3.0,
                               readonly=False, group='tuning')
    ramp_timeout = Parameter('timeout for reaching the commanded speed',
                             FloatRange(0, unit='s'), default=20.0,
                             readonly=False, group='tuning')
    stop_timeout = Parameter('timeout for coming to a standstill',
                             FloatRange(0, unit='s'), default=10.0,
                             readonly=False, group='tuning')
    speed_tolerance = Parameter('speed deviation still counted as on target',
                                FloatRange(0, unit='rpm'), default=2.0,
                                readonly=False, group='tuning')
    halt_toggle = Parameter(
        'pulse the halt bit before Enable Operation when latching a new target,'
        ' so gateways that only forward changed registers still see an edge',
        datatypes.BoolType(), default=True, readonly=False, group='tuning')

    _blocking_fault = None

    # -- driving -----------------------------------------------------------
    def write_target(self, value):
        if self._blocking_fault:
            raise HardwareError(
                f'{self._blocking_fault} - run reset_fault first')
        self.start_machine(self.enabling, target=value)
        self._latch_setpoint()
        return value

    @status_code(300, 'enabling drive')  # 300 == BUSY
    def enabling(self, sm):
        if self._fields is None:
            return Retry  # no block read yet
        if sm.init:
            self._write('write_mode_of_operation',
                        int(OperatingMode.PROFILE_VELOCITY))
            sm.deadline = sm.now + self.enable_timeout
        sw = self._drv('read_status_word')
        if StatusControl.is_enabled(sw):
            # 0x60FF is only the active set-point while PVM is the running
            # mode; in any other mode the drive sits enabled and still, with
            # velocity_demand stuck away from target. Re-assert rather than
            # assume the write at sm.init landed.
            mode = self._drv('read_mode_of_operation') & 0xFF
            if mode != OperatingMode.PROFILE_VELOCITY:
                self._write('write_mode_of_operation',
                            int(OperatingMode.PROFILE_VELOCITY))
                if sm.now > sm.deadline:
                    raise HardwareError(
                        f'drive is enabled but stayed in mode {mode} instead of '
                        f'{int(OperatingMode.PROFILE_VELOCITY)} (profile '
                        f'velocity) - target velocity will be ignored. Check '
                        f'that write_mode_of_operation is mapped correctly.')
                return Retry
            return self.ramp_to_speed
        cw = StatusControl.next_controlword(sw)
        if cw is not None:
            self._write_controlword(cw)
        if sm.now > sm.deadline:
            _, state = StatusControl.get_state(sw)
            raise HardwareError(
                f'drive did not enable, stuck in {DRIVE_STATE_TEXT[state]!r} '
                f'after {self.enable_timeout:g} s. {self._enable_diagnosis(sw)}')
        return Retry

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

    def _latch_setpoint(self):
        """Make the drive act on the target velocity just written.

        The EPOS4 picks up a new PVM set-point when a controlword arrives, not
        when 0x60FF is written - so a target on its own leaves the profile
        generator idle and velocity_demand pinned. Re-sending Enable Operation
        is what starts it.

        The controlword usually already holds 0x000F at this point, and a
        gateway that only forwards an SDO when the mapped register *changes*
        would swallow a plain repeat. Toggling the halt bit guarantees the
        drive sees a real transition. Set `halt_toggle = False` if your gateway
        forwards every write and you would rather avoid the momentary halt.
        """
        if self.halt_toggle:
            self._write_controlword(ControlWord.ENABLE_OPERATION_HALT)
        self._write_controlword(ControlWord.ENABLE_OPERATION)

    @status_code(300, 'ramping to speed')
    def ramp_to_speed(self, sm):
        if sm.init:
            self._write('write_target_velocity', self._rpm_to_raw(sm.target))
            # target first, then the controlword that latches it
            self._latch_setpoint()
            sm.deadline = sm.now + self.ramp_timeout
            # The statusword we hold was read before the new set-point was
            # written, so its TARGET_REACHED bit still refers to the previous
            # target. Wait for a fresh one rather than believe it.
            return Retry
        sw = self._drv('read_status_word')
        if StatusControl.is_fault(sw):
            raise HardwareError(
                f'drive faulted while ramping '
                f'(error 0x{self._drv("read_error_code"):04X})')
        if not StatusControl.is_enabled(sw):
            return self.enabling  # dropped out, re-sequence
        speed = self._raw_to_rpm(self._drv('read_velocity_value'))
        if (sw & StatusWord.TARGET_REACHED
                or abs(speed - sm.target) <= self.speed_tolerance):
            if sm.target:
                return self.final_status(IDLE, f'rotating at {sm.target:g} rpm')
            return self.final_status(IDLE, 'stopped')
        if sm.now > sm.deadline:
            raise HardwareError(
                f'did not reach {sm.target:g} rpm within {self.ramp_timeout:g} s '
                f'(at {speed:g} rpm)')
        return Retry

    @status_code(300, 'stopping')
    def stopping(self, sm):
        if sm.init:
            self._write_controlword(ControlWord.ENABLE_OPERATION_HALT)
            sm.deadline = sm.now + self.stop_timeout
            # as in ramp_to_speed: the statusword in hand predates the halt
            return Retry
        sw = self._drv('read_status_word')
        speed = self._raw_to_rpm(self._drv('read_velocity_value'))
        if sw & StatusWord.TARGET_REACHED or abs(speed) <= self.speed_tolerance:
            return self.final_status(IDLE, 'stopped')
        if sm.now > sm.deadline:
            return self.final_status(
                WARN, f'still turning at {speed:g} rpm after halt')
        return Retry

    @Command
    def stop(self):
        """Ramp down to standstill, leaving the power stage enabled."""
        self.target = 0
        self.start_machine(self.stopping, target=0)

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
        """Merge CiA-402 drive faults with the state machine's status."""
        sw = self._field('read_status_word')
        code, state = StatusControl.get_state(sw)
        if code >= ERROR:
            self._blocking_fault = (
                f'{DRIVE_STATE_TEXT[state]} '
                f'(error 0x{self._field("read_error_code"):04X})')
            return code, self._blocking_fault
        if self._blocking_fault:
            return ERROR, f'{self._blocking_fault} - run reset_fault'
        machine = super().read_status()  # HasStates -> state machine status
        if machine[0] == IDLE and state != DriveState.OPERATION_ENABLED:
            # idle, but the drive is not actually under power
            return WARN, DRIVE_STATE_TEXT[state]
        return machine
