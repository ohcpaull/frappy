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
specific to the tumbler belongs here. The rotation axis is commanded through
an analog output voltage rather than CANopen 0x60FF, because of a shroud
interlock that must be able to stop and restart it without any intervention
from this driver - see :class:`frappy_acns.epos4.EPOS4Velocity` for the full
explanation of why, and why that class must never send a controlword to
resume motion on this axis.

The interlock and the ``stop`` command act through different mechanisms, which
is worth knowing when reading the status: the interlock gates the drive's
'Drive enable' digital input, while ``stop`` leaves the drive enabled and
commands zero speed on the analog output.
"""

from .epos4 import EPOS4Velocity


class TumblerRotation(EPOS4Velocity):
    """Continuous rotation of a tumbler axis, via analog-voltage CSV.

    Configure one module per drive, all sharing a single ``ModbusIO``, each
    with the ``input_offset`` / ``holding_offset`` of its register window in
    the gateway, and with ``zero_speed_voltage`` / ``rpm_per_volt`` /
    ``output_voltage_min`` / ``output_voltage_max`` set to this axis's actual
    calibration if it differs from the EPOS4Velocity defaults.
    """
