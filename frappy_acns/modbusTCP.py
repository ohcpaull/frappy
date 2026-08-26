from threading import Lock

from frappy.core import Parameter, HasIO, StringIO, Property, StringType, \
    IDLE, BUSY, WARN, ERROR, Writable, Drivable, IntRange, Attached, \
    StatusType
from frappy.lib import parse_host_port

from frappy.modules import Drivable, Communicator, Property, Parameter
from frappy import datatypes
from frappy.errors import CommunicationFailedError
#from frappy_acns import MeComIO
from pyModbusTCP.client import ModbusClient
from dataclasses import dataclass
from enum import Enum, IntEnum, IntFlag


# ==============================================================================
# Communicators (Low-level Hardware Interfaces)
# ==============================================================================

class DataType(Enum):
    I8 = "i8"
    U16 = "u16"
    U32 = "u32"
    I16 = "i16"
    I32 = "i32"


@dataclass(frozen=True)
class RegisterDef:
    datatype: DataType
    encode: callable
    decode: callable
    address: int

    tag: str | None = None
    length: int = 1
    writable: bool = False
    scalefactor: float = 1.0
   

class EPOSModbusIO(Communicator):
    """Handles communication with the CANopen gateway controlling the rotation motor."""
    uri = Property("""uri for serial connection

                   one of the following:

                   - ``tcp://<host address>:<portnumber>`` (see :class:`frappy.lib.asynconn.AsynTcp`)

                   - ``serial://<serial device>?baudrate=<value>...`` (see :class:`frappy.lib.asynconn.AsynSerial`)
                   """, datatype=StringType())
    pollinterval = 1

    def initModule(self):
        super().initModule()
        # Initialize your socket/CANopen driver interface here
        # e.g., self.can_conn = connect_can(self.ip_address, self.port)
        if self.uri.startswith('tcp://'):
            uri = self.uri[6:]
        host, port = parse_host_port(uri)
        try:
            self._conn = ModbusClient(host=host, port=port)
        except ValueError:
            raise CommunicationFailedError(f"Cannot connect to IP {self.ip_address} on port {self.port}")

    def connect(self):
        if not self._conn.is_open:
            self._conn.open()

    @classmethod
    def registers_to_u16(self, reg):
        return reg

    @classmethod
    def u16_to_registers(self, value):
        return value

    @classmethod
    def registers_to_u32(self, reg):
        """Combine two 16-bit Modbus registers into a 32-bit unsigned integer."""
        reg_hi = reg[0]
        reg_lo = reg[1]
        return (reg_hi << 16) | reg_lo

    @classmethod
    def u32_to_registers(self, value):
        """Split a 32-bit unsigned integer into two 16-bit Modbus registers."""
        return [
            (value >> 16) & 0xFFFF,  # high word
            value & 0xFFFF           # low word
        ]

    @classmethod
    def registers_to_i32(self, reg: list) -> int:
        """Combine two 16-bit Modbus registers into a 32-bit signed integer."""
        reg_hi = reg[0]
        reg_lo = reg[1]
        value = (reg_hi << 16) | reg_lo
        if value & 0x80000000:
            value -= 0x100000000
        return value

    @classmethod
    def i32_to_registers(self, value):
        """Split a 32-bit signed integer into two 16-bit Modbus registers."""
        value &= 0xFFFFFFFF
        return [
            (value >> 16) & 0xFFFF,
            value & 0xFFFF
        ]

    def communicate(self, register: RegisterDef):
        """Send Modbus request"""

        self.connect()

        #dtype = READ_REGISTER_MAP[register]
        response_registers = self._conn.read_input_registers(register.address, register.length)
        self.log.info(f"response: {response_registers}")
        response = register.decode(response_registers)
        
        return response

    def set(self, register, data):

        self.connect()

        #dtype = WRITE_REGISTER_MAP[register]
        response_registers = self._conn.write_multiple_registers(register.address, register.encode(data))
        return response_registers