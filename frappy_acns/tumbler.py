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
"""The Tumbler.

Maxon EPOS4 motors provide the rotation and talk CANopen; they are connected to
a Moxa Mport 5121 CANopen -> ModbusTCP gateway, one register window per drive.
Meerstetter TEC devices provide the temperature control and talk MeCom - those
modules reuse ``MeComIO`` / ``TECBase`` from :mod:`frappy_acns.sample_changer`
and are not part of this file yet.

All EPOS4/CiA-402 behaviour lives in :mod:`frappy_acns.epos4`; only what is
specific to the tumbler belongs here.
"""

from frappy.core import FloatRange, Parameter

from .epos4 import EPOS4Motion


class TumblerRotation(EPOS4Motion):
    """Continuous rotation of a tumbler axis (EPOS4 profile velocity mode).

    Configure one module per drive, all sharing a single ``ModbusIO``, each
    with the ``read_offset`` / ``write_offset`` of its register window in the
    gateway.
    """
    target = Parameter('commanded tumbling speed',
                       FloatRange(-500, 500, unit='rpm'), readonly=False,
                       default=0)
    value = Parameter('actual tumbling speed',
                      FloatRange(-500, 500, unit='rpm'))
