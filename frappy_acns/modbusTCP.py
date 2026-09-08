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
"""Generic ModbusTCP communicator.

Devices are described by a table of :class:`RegisterDef` entries. Addresses in
such a table are *relative* to the device's own window in the server's register
map, so several identical devices behind one gateway can share one table and be
distinguished by the ``offset`` passed to :meth:`ModbusIO.communicate` /
:meth:`ModbusIO.set`.
"""

from dataclasses import dataclass
from threading import Lock
from typing import Callable

from frappy.core import Property, StringType
from frappy.lib import parse_host_port
from frappy.modules import Communicator
from frappy import datatypes
from frappy.errors import CommunicationFailedError

from pyModbusTCP.client import ModbusClient
from pyModbusTCP.constants import MB_EXCEPT_ERR


# ==============================================================================
# Communicators (Low-level Hardware Interfaces)
# ==============================================================================


@dataclass(frozen=True)
class RegisterDef:
    """Describes one logical value in a Modbus register map.

    Values are decoded raw; engineering-unit scaling belongs to the device
    module (it is deployment specific - encoder resolution, gearing, the drive's
    velocity notation index - not a property of the register itself).

    :param address: register address, relative to the device's window
    :param length: number of 16-bit registers the value occupies
    """
    datatype: datatypes.DataType
    encode: Callable
    decode: Callable
    address: int

    tag: str | None = None
    length: int = 1
    writable: bool = False


class ModbusIO(Communicator):
    """Handles ModbusTCP communication, e.g. with a CANopen <-> ModbusTCP gateway."""
    uri = Property("""uri for the connection

                   ``tcp://<host address>:<portnumber>``
                   """, datatype=StringType())
    pollinterval = Property('reconnect/poll interval', datatypes.FloatRange(0.1, 120),
                            default=1)

    _conn = None
    _lock = None

    def initModule(self):
        super().initModule()

        if self.uri.startswith('tcp://'):
            uri = self.uri[6:]
        host, port = parse_host_port(uri)
        try:
            self._conn = ModbusClient(host=host, port=port,
                                      auto_open=True, auto_close=False)
        except ValueError:
            raise CommunicationFailedError(f"Cannot connect to IP {self.ip_address} on port {self.port}")


        #uri = self.uri[6:] if self.uri.startswith('tcp://') else self.uri
        #host, port = parse_host_port(uri)
        # auto_open lets pyModbusTCP (re)open the socket lazily on the first
        # request and transparently after a drop, so there is no need for an
        # explicit connect() check on every transaction.
        #self._conn = ModbusClient(host=host, port=port,
        #                          auto_open=True, auto_close=False)
        # pyModbusTCP sockets are not safe for concurrent use; the frappy poller
        # and command handlers run on different threads, so every transaction is
        # serialised through this lock.
        self._lock = Lock()

    @staticmethod
    def registers_to_u16(reg):
        # read_input_registers always hands back a list, even for a single
        # register - callers expect the scalar value.
        return reg[0]

    @staticmethod
    def u16_to_registers(value):
        return [value & 0xFFFF]

    @staticmethod
    def registers_to_u32(reg):
        """Combine two 16-bit Modbus registers into a 32-bit unsigned integer."""
        reg_hi = reg[0]
        reg_lo = reg[1]
        return (reg_hi << 16) | reg_lo

    @staticmethod
    def u32_to_registers(value):
        """Split a 32-bit unsigned integer into two 16-bit Modbus registers."""
        return [
            (value >> 16) & 0xFFFF,  # high word
            value & 0xFFFF           # low word
        ]

    @staticmethod
    def registers_to_i32(reg: list) -> int:
        """Combine two 16-bit Modbus registers into a 32-bit signed integer."""
        reg_hi = reg[0]
        reg_lo = reg[1]
        value = (reg_hi << 16) | reg_lo
        if value & 0x80000000:
            value -= 0x100000000
        return value

    @staticmethod
    def i32_to_registers(value):
        """Split a 32-bit signed integer into two 16-bit Modbus registers."""
        value &= 0xFFFFFFFF
        return [
            (value >> 16) & 0xFFFF,
            value & 0xFFFF
        ]

    def _why(self):
        """Describe the last failure, naming the exception code if the server
        sent one (a bare 'modbus exception' hides the useful half)."""
        conn = self._conn
        if conn.last_error == MB_EXCEPT_ERR:
            return (f'exception {conn.last_except} '
                    f'({conn.last_except_as_txt}) - {conn.last_except_as_full_txt}')
        return conn.last_error_as_txt

    def fetch_block(self, address, length):
        """Read a contiguous span of input registers in one Modbus transaction.

        `address` is absolute. Returns the raw register list; callers slice
        individual fields out of it and decode them, so a whole poll cycle costs
        a single round-trip instead of one per parameter. Raises
        CommunicationFailedError on a failed read.

        Note that Modbus requires *every* register in the span to exist on the
        server: one unmapped address anywhere in the range makes the whole
        request fail with 'illegal data address', even though each register
        would read fine on its own.
        """
        with self._lock:
            regs = self._conn.read_input_registers(address, length)
        if regs is None:
            raise CommunicationFailedError(
                f"modbus read of {length} register(s) at {address}"
                f"..{address + length - 1} failed: {self._why()}")
        return regs

    def communicate(self, register: RegisterDef, offset=0):
        """Read and decode a single register definition.

        Returns the raw decoded value; any engineering-unit scaling is the
        caller's business, so that this path and the block-read path used by
        polling cannot disagree.

        :param offset: base address of the device's window in the register map
        """
        address = register.address + offset
        regs = self.fetch_block(address, register.length)
        self.log.debug('read %s @%d -> %r', register.tag, address, regs)
        return register.decode(regs)

    def set(self, register: RegisterDef, data, offset=0):
        """Encode and write a single register definition.

        :param offset: base address of the device's window in the register map
        """
        address = register.address + offset
        with self._lock:
            ok = self._conn.write_multiple_registers(
                address, register.encode(data))
        if not ok:
            raise CommunicationFailedError(
                f"modbus write to {register.tag} @{address} failed: "
                f"{self._why()}")
        return ok
