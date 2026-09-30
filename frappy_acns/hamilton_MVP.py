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
'''
This module contains driver code for a Hamilton Multi-Valve Positioner (MVP) for
use with a HPLC pump in neutron reflectometry experiments (typically soft matter).

These need a Moxa 5150 with 9600 baud, odd parity, 7 data bits, 1 stop bit, and 1 start bit.

This driver uses the RNO+ syntax protocol. The configuration of the MVPs is such that there
are 6 positions that are rotated by 60 degrees. 

'''
from frappy.core import Readable, Writable, Drivable, StringIO, HasIO, IntRange, \
    FloatRange, StringType, Parameter, BUSY, IDLE, ERROR

rot = {
    "CLOCKWISE": "LP0",
    "ANTICLOCKWISE": "LP1"
}

class HamiltonIO(StringIO):
    end_of_line = '\r'

    identification = ['1a', '1b']


class HamiltonBase(HasIO):
    ioClass = HamiltonIO

    address = Parameter('Address of MVP', datatype=StringType(default='a'))


class HamiltonMVP(HamiltonBase, Drivable):
    value = Parameter('Actual position', datatype=IntRange(min=1,max=6))
    target = Parameter('Target position', datatype=IntRange(min=1,max=6))

    def onInit(self):
        reply = self.communicate(f'{self.address}LX')
        return reply

    def read_value(self):
        reply = self.communicate(f'{self.address}LQP')
        return reply
    
    def write_value(self):
        rotation = self._determine_rotation()
        self.communicate(f'{self.address}{rotation}{self.target}R')

    def _determine_rotation(self):
        if self.value < self.target:
            rotation = "LP0" # Clockwise
        else: 
            rotation = "LP1" # Anti-clockwise

    def read_status(self):
        movement = self.communicate(f'{self.address}F')
        valve = self.communicate(f'{self.address}G')

        if valve == 'Y':
            return ERROR, 'Valve overload'
        if movement == 'N':
            return BUSY, 'Command queued'
        elif movement == 'Y':
            return IDLE, 'Ready'
        elif movement == '*':
            return BUSY, 'Driving...'

        