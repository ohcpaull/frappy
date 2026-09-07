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

from frappy.core import Parameter, HasIO, Readable, FloatRange

from frappy.modules import Parameter
from frappy import datatypes
from .modbusTCP import ModbusIO, RegisterDef


TAG_MAP = {
    "read_temp": RegisterDef(
        address = 1,
        datatype=datatypes.UInt16, 
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "read_setpoint": RegisterDef(
        address = 2,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "read_ramp_setpoint": RegisterDef(
        address = 21,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "read_output_power": RegisterDef(
        address = 3,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "read_output_power": RegisterDef(
        address = 3,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
}

class West4100(HasIO, Readable):

    ioClass = ModbusIO

    temperature = Parameter('Temperature of furnace probe', FloatRange())
    setpoint = Parameter('Setpoint temperature', FloatRange())
    ramp_setpoint = Parameter('Working setpoint', FloatRange())
    output_power = Parameter('Power output in %', FloatRange(min=0, max=100))

    def read_temperature(self):
        response = self.communicate(TAG_MAP["read_temp"])[0]
        return response

    def read_setpoint(self):
        response = self.communicate(TAG_MAP["read_setpoint"])[0]
        return response

    def read_ramp_setpoint(self):
        response = self.communicate(TAG_MAP["read_ramp_setpoint"])[0]
        return response

    def read_output_power(self):
        response = self.communicate(TAG_MAP["read_output_power"])[0]
        return response