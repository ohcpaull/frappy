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
    "read_pastis_state": RegisterDef(
        address = 41061,
        datatype=datatypes.BoolType,
        encode=ModbusIO.u16_to_registers,
        decode=ModbusIO.registers_to_u16),
    "read_field": RegisterDef(
        address = 41063,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers,
        decode=ModbusIO.registers_to_u16),
    "read_theta": RegisterDef(
        address = 41064,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers,
        decode=ModbusIO.registers_to_u16),
    "read_phi": RegisterDef(
        address = 41065,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers,
        decode=ModbusIO.registers_to_u16),
    "read_magic_box_1_state": RegisterDef(
        address = 41071,
        datatype=datatypes.BoolType,
        encode=ModbusIO.u16_to_registers,
        decode=ModbusIO.registers_to_u16),
    "read_magic_box_2_state": RegisterDef(
        address = 41072,
        datatype=datatypes.BoolType,
        encode=ModbusIO.u16_to_registers,
        decode=ModbusIO.registers_to_u16),
    "write_pastis_enable": RegisterDef(
        address = 41001,
        datatype=datatypes.BoolType, 
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_pastis_go": RegisterDef(
        address = 41002,
        datatype=datatypes.BoolType,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_pastis_reset": RegisterDef(
        address = 41003,
        datatype=datatypes.BoolType,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_target_gauss": RegisterDef(
        address = 41004,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_target_theta": RegisterDef(
        address = 41005,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_target_phi": RegisterDef(
        address = 41006,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_gauss_ramp_rate": RegisterDef(
        address = 41007,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_angular_ramp_rate": RegisterDef(
        address = 41008,
        datatype=datatypes.UInt16,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_magic_box_1_state": RegisterDef(
        address = 41031,
        datatype=datatypes.BoolType,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_magic_box_2_state": RegisterDef(
        address = 41032,
        datatype=datatypes.BoolType,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_spin_flipper_1": RegisterDef(
        address = 41041,
        datatype=datatypes.BoolType,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
    "write_spin_flipper_2": RegisterDef(
        address = 41042,
        datatype=datatypes.BoolType,
        encode=ModbusIO.u16_to_registers, 
        decode=ModbusIO.registers_to_u16),
}


class PastisCoils(HasIO, Readable):

    ioClass = ModbusIO

    magic_box_1 = Parameter('Magic Box 1 state', datatypes.BoolType())
    magic_box_2 = Parameter('Magic Box 2 state', datatypes.BoolType())
    spin_flipper_1 = Parameter('Spin flipper 1 state', datatypes.BoolType())
    spin_flipper_2 = Parameter('Spin flipper 2 state', datatypes.BoolType())

    field = Parameter('Magnetic field in Gauss', FloatRange(units='G'))
    theta = Parameter('Theta of magnetic field', FloatRange(units='deg'))
    phi = Parameter('Phi of magnetic field', FloatRange(units='deg'))



    setpoint = Parameter('Setpoint temperature', FloatRange())
    ramp_setpoint = Parameter('Working setpoint', FloatRange())
    output_power = Parameter('Power output in %', FloatRange(min=0, max=100))


    def doPoll(self):
        gauss = self.communicate(TAG_MAP["read_field"])
        self.field = gauss/1000

        angle_theta = self.communicate(TAG_MAP["read_theta"])
        self.theta = angle_theta / 100

        angle_phi = self.communicate(TAG_MAP["read_phi"])
        self.phi = angle_phi / 100

        
    def read_field(self):
        response = self.communicate(TAG_MAP["read_temp"])[0]
        return response
