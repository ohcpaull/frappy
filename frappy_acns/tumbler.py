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
''' The Tumbler consists of Maxon EPOS4 motors (which facilitate rotation) that talk CANopen, 
and Meerstetter TEC devices (which facilitate temperature control) that talk Mecom. The EPOS4 
motors are connected to a Moxa Mport 5121 CANopen -> ModbusTCP gateway. '''
from frappy.core import Parameter, HasIO, StringIO, Property, StringType, \
    IDLE, WARNING, BUSY, WARN, ERROR, Writable, Drivable, IntRange, Attached, \
    StatusType
from frappy.lib import parse_host_port

from frappy.modules import Drivable, Communicator, Property, Parameter
from frappy import datatypes
from frappy.errors import CommunicationFailedError
#from frappy_acns import MeComIO
from pyModbusTCP.client import ModbusClient
from dataclasses import dataclass
from enum import Enum, IntEnum, IntFlag

from .modbusTCP import RegisterDef, EPOSModbusIO


class StatusWord(IntFlag):
    ''' Class to define the Statusword bit patterns for use in StatusControl'''
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
    ''' Class to define the Controlword bit patterns for use in StatusControl'''
    DISABLE = 0x0000
    READY = 0x000F
    READY_HALT = 0x010F
    QUICKSTOP = 0x0002
    FAULT_RESET = 0x0080


class DriveState(Enum):
    NOT_READY = "not ready to switch on"
    SWITCH_ON_DISABLED = "switch on disabled"
    READY_TO_SWITCH_ON = "ready to switch on"
    SWITCHED_ON = "switched on"
    OPERATION_ENABLED = "operation enabled"
    QUICK_STOP_ACTIVE = "quick stop active"
    FAULT = "fault"


@dataclass(frozen=True)
class Status:
    ready_to_switch_on: bool
    switched_on: bool
    operation_enabled: bool
    fault: bool
    voltage_enabled: bool
    quick_stop: bool
    switch_on_disabled: bool
    warning: bool
    remote: bool
    target_reached: bool
    internal_limit: bool


class StatusControl:
    """Helper class for decoding EPOS/CiA-402 statuswords and generating controlwords."""

    @staticmethod
    def decode_statusword(sw: int) -> Status:
        status = StatusWord(sw)

        return Status(
            ready_to_switch_on=bool(status & StatusWord.READY_TO_SWITCH_ON),
            switched_on=bool(status & StatusWord.SWITCHED_ON),
            operation_enabled=bool(status & StatusWord.OPERATION_ENABLED),
            fault=bool(status & StatusWord.FAULT),
            voltage_enabled=bool(status & StatusWord.VOLTAGE_ENABLED),
            quick_stop=bool(status & StatusWord.QUICK_STOP),
            switch_on_disabled=bool(status & StatusWord.SWITCH_ON_DISABLED),
            warning=bool(status & StatusWord.WARNING),
            remote=bool(status & StatusWord.REMOTE),
            target_reached=bool(status & StatusWord.TARGET_REACHED),
            internal_limit=bool(status & StatusWord.INTERNAL_LIMIT),
        )

    @staticmethod
    def get_state(sw: int) -> DriveState:
        """
        Decode CiA-402 state from statusword.

        Bit patterns based on the DS402 state machine.
        """

        if (sw & 0x004F) == 0x0008:
            return ERROR, DriveState.FAULT

        if (sw & 0x006F) == 0x0007:
            return WARNING, DriveState.QUICK_STOP_ACTIVE

        if (sw & 0x006F) == 0x0027:
            return IDLE, DriveState.OPERATION_ENABLED

        if (sw & 0x006F) == 0x0023:
            return IDLE, DriveState.SWITCHED_ON

        if (sw & 0x006F) == 0x0021:
            return IDLE, DriveState.READY_TO_SWITCH_ON

        if (sw & 0x004F) == 0x0040:
            return WARNING, DriveState.SWITCH_ON_DISABLED

        return WARNING, DriveState.NOT_READY

    @staticmethod
    def is_fault(sw: int) -> bool:
        return bool(sw & StatusWord.FAULT)

    @staticmethod
    def is_enabled(sw: int) -> bool:
        return StatusControl.get_state(sw) == DriveState.OPERATION_ENABLED

    @staticmethod
    def fault_reset_word() -> int:
        return ControlWord.FAULT_RESET

    @staticmethod
    def enable_operation_word() -> int:
        return ControlWord.READY

    @staticmethod
    def disable_operation_word() -> int:
        return ControlWord.DISABLE

    @staticmethod
    def quickstop_word() -> int:
        return ControlWord.QUICKSTOP



TAG_MAP = {
    "read_status_word": RegisterDef(
        address = 0,
        datatype=DataType.U16, 
        encode=EPOSModbusIO.u16_to_registers, 
        decode=EPOSModbusIO.registers_to_u16, 
        tag="Statusword"),
    "read_control_word": RegisterDef(
        address = 1,
        datatype=DataType.U16,
        encode=EPOSModbusIO.u16_to_registers, 
        decode=EPOSModbusIO.registers_to_u16, 
        tag="Controlword"),
    "read_error_code": RegisterDef(
        address = 2,
        datatype=DataType.U16,  
        encode=EPOSModbusIO.u16_to_registers, 
        decode=EPOSModbusIO.registers_to_u16, 
        tag="Error_code"),
    "read_mode_of_operation": RegisterDef(
        address=3, 
        datatype=DataType.U16, 
        encode=EPOSModbusIO.u16_to_registers, 
        decode=EPOSModbusIO.registers_to_u16, 
        tag="Mode_of_operation_display"),
    "read_position_value": RegisterDef(
        address=4,
        datatype=DataType.I32, 
        length=2, 
        encode=EPOSModbusIO.i32_to_registers, 
        decode=EPOSModbusIO.registers_to_i32, 
        tag="Position_actual_value"),
    "read_position_demand" : RegisterDef(
        address=6,
        datatype=DataType.I32,
        length=2, 
        encode=EPOSModbusIO.i32_to_registers, 
        decode=EPOSModbusIO.registers_to_i32, 
        tag="Position_demand_value"),
    "read_velocity_value": RegisterDef(
        address=8,
        datatype=DataType.I32, 
        length=2, 
        encode=EPOSModbusIO.i32_to_registers, 
        decode=EPOSModbusIO.registers_to_i32, 
        tag="Velocity_actual_value_averaged"),
    "read_velocity_demand": RegisterDef(
        address=10,
        datatype=DataType.I32, 
        length=2, 
        encode=EPOSModbusIO.i32_to_registers, 
        decode=EPOSModbusIO.registers_to_i32, 
        tag="Velocity_demand_value"),
    "read_current_value": RegisterDef(
        address=12,
        datatype=DataType.I32, 
        length=2, 
        encode=EPOSModbusIO.i32_to_registers, 
        decode=EPOSModbusIO.registers_to_i32, 
        tag="Current_actual_value_averaged"),
    "read_torque_value": RegisterDef(
        address=14,
        datatype=DataType.I32, 
        length=2,
        encode=EPOSModbusIO.i32_to_registers,
        decode=EPOSModbusIO.registers_to_i32,
        tag="Torque_actual_value_averaged"),
    "write_control_word": RegisterDef(
        address=0,
        datatype=DataType.U16,
        encode=EPOSModbusIO.u16_to_registers,
        decode=EPOSModbusIO.registers_to_u16,
        tag="Controlword"),
    "write_mode_of_operation": RegisterDef(
        address=1,
        datatype=DataType.I8,
        encode=EPOSModbusIO.u16_to_registers,
        decode=EPOSModbusIO.registers_to_u16,
        tag="Modes_of_operation"),
    "write_target_position": RegisterDef(
        address=2,
        datatype=DataType.I32,
        length=2,
        encode=EPOSModbusIO.i32_to_registers,
        decode=EPOSModbusIO.registers_to_i32,
        tag="Target_position"),
    "write_target_velocity": RegisterDef(
        address=4,
        datatype=DataType.I32,
        length=2,
        encode=EPOSModbusIO.i32_to_registers,
        decode=EPOSModbusIO.registers_to_i32,
        tag="Target_velocity")
}
#class EPOSBase(HasIO):

# Define datatype of linked Modbus registers, and how to encode/decode information when sending/receiving data
READ_REGISTER_MAP = {
    0: RegisterDef(DataType.U16, encode=EPOSModbusIO.u16_to_registers, decode=EPOSModbusIO.registers_to_u16, tag="Statusword"),
    1: RegisterDef(DataType.U16, encode=EPOSModbusIO.u16_to_registers, decode=EPOSModbusIO.registers_to_u16, tag="Controlword"),
    2: RegisterDef(DataType.U16, encode=EPOSModbusIO.u16_to_registers, decode=EPOSModbusIO.registers_to_u16, tag="Error_code"),
    3: RegisterDef(DataType.I8, encode=EPOSModbusIO.u16_to_registers, decode=EPOSModbusIO.registers_to_u16, tag="Modes_of_operation_display"),
    4: RegisterDef(DataType.I32, length=2, encode=EPOSModbusIO.i32_to_registers, decode=EPOSModbusIO.registers_to_i32, tag="Position_actual_value"),
    6: RegisterDef(DataType.I32, length=2, encode=EPOSModbusIO.i32_to_registers, decode=EPOSModbusIO.registers_to_i32, tag="Position_demand_value"),
    8: RegisterDef(DataType.I32, length=2, encode=EPOSModbusIO.i32_to_registers, decode=EPOSModbusIO.registers_to_i32, tag="Velocity_actual_value_averaged"),
    10: RegisterDef(DataType.I32, length=2, encode=EPOSModbusIO.i32_to_registers, decode=EPOSModbusIO.registers_to_i32, tag="Velocity_demand_value"),
    12: RegisterDef(DataType.I32, length=2, encode=EPOSModbusIO.i32_to_registers, decode=EPOSModbusIO.registers_to_i32, tag="Current_actual_value_averaged"),
    14: RegisterDef(DataType.I32, length=2, encode=EPOSModbusIO.i32_to_registers, decode=EPOSModbusIO.registers_to_i32, tag="Torque_actual_value_averaged"),
}

WRITE_REGISTER_MAP = {
    0: RegisterDef(DataType.U16, encode=EPOSModbusIO.u16_to_registers, decode=EPOSModbusIO.registers_to_u16, tag="Controlword"),
    1: RegisterDef(DataType.I8, encode=EPOSModbusIO.u16_to_registers, decode=EPOSModbusIO.registers_to_u16, tag="Modes_of_operation"),
    2: RegisterDef(DataType.I32, encode=EPOSModbusIO.i32_to_registers, decode=EPOSModbusIO.registers_to_i32, tag="Target_position"),
    4: RegisterDef(DataType.I32, encode=EPOSModbusIO.i32_to_registers, decode=EPOSModbusIO.registers_to_i32, tag="Target_velocity")
}


# ==============================================================================
# SECoP Modules (High-level Equipment Logic)
# ==============================================================================
class TumblerBase(HasIO, Writable):
    ioClass = EPOSModbusIO

    state = Attached(Status)

    status = Parameter(StatusType())



class TumblerRotation(TumblerBase, Drivable):
    """Controls the 8-position rotation mechanism via CANopen."""
    
    # Attach the specific CANopen communicator
    ioClass = EPOSModbusIO
    
    # Standard SECoP Drivable targets and parameters
    target = Parameter("Target sample position (1-8)", datatypes.IntRange())
    position = Parameter("Current sample position", datatypes.IntRange(), readonly=True)
    velocity = Parameter("Rotation speed in RPM", datatypes.FloatRange(), unit="rpm")

    # Mapping target position (1-8) to physical encoder counts or angles
    POS_TICKS = {1: 0, 2: 45, 3: 90, 4: 135, 5: 180, 6: 225, 7: 270, 8: 315}

    def read_position(self):
        response = self.communicate(
            TAG_MAP["read_position_actual"]
        )
        return response

    def read_velocity(self):
        response = self.communicate(
            TAG_MAP["read_velocity_actual"]
        )
        return response

    def read_status(self):
        response = self.communicate(
            TAG_MAP["read_status_word"]
        )
        return StatusControl.get_state(response)







'''
class TumblerTemperature(Drivable):
    """Controls temperature of the tumbler positions via Meerstetter TEC."""
    
    # Attach the specific MeCom communicator
    com = Attached("MeCom communicator module", MeComIO)

    target = Parameter("Target temperature", datatypes.FloatRange(-20.0, 100.0), unit="degC")
    value = Parameter("Current temperature", datatypes.FloatRange(-20.0, 100.0), readonly=True, unit="degC")

    # Meerstetter Parameter IDs (MeCom object IDs)
    PARAM_OBJECT_TEMP = 1000   # Example: Object Temperature
    PARAM_TARGET_TEMP = 3000   # Example: Target Temperature Setpoint

    def doStart(self, target_temp):
        self.target = target_temp
        # Write setpoint to Meerstetter controller via communicator
        self.com.set_parameter(self.PARAM_TARGET_TEMP, target_temp)
        self.status = (Status.BUSY, f"Changing temperature setpoint to {target_temp} degC")

    def doPoll(self):
        """Poll current temperature from Meerstetter controller."""
        try:
            current_temp = self.com.get_parameter(self.PARAM_OBJECT_TEMP)
            self.value = round(float(current_temp), 2)

            if self.target is not None and abs(self.value - self.target) <= 0.2:
                self.status = (Status.IDLE, "Temperature stable")
            else:
                self.status = (Status.BUSY, "Reaching target temperature")
        except Exception as e:
            self.status = (Status.ERROR, f"MeCom read failed: {str(e)}")
'''
