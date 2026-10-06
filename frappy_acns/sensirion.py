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
Module for communication with Sensirion mass-flow controllers. These come in 
different types that have different maximum flow-rates for gases. Communication
is comprised of frames from the master to slave (MOSI frame) and from the slave
to the master (MISO). 

MOSI Frames are constructed as

<Start> <Address> <CMD> <Length> <Tx Data> <CHK> <Stop>

<Start> - 1 byte - 0x7E
<Address> - 1 byte - 0 to 255
<CMD> - 1 byte - Command byte which tells teh device what to do with the given data
<Length> - 1 byte - Length of the <Tx Data> field (before byte stuffing)
<Tx Data> - N bytes
<CHK> - 1 byte - Checksum of frame content (everything between the Start and Stop bytes)
<Stop> - 1 byte - 0x7E

MISO response frames are similar in construction but include a status byte

<Start> <Address> <CMD> <State> <Length> <Rx Data> <CHK> <Stop>

<State> - 1 byte - bit 7: device error flag (details with command 0xD2),
                   bits 0-6: execution error code (see ErrorCode)

Reference: Sensirion SFC5xxx SHDLC Interface Reference V1.9
'''
from frappy.core import BytesIO, HasIO, Drivable, Parameter, FloatRange, IDLE, WARN, ERROR
from frappy import datatypes
from frappy.errors import CommunicationFailedError, HardwareError
from dataclasses import dataclass, replace
import struct
from enum import IntEnum

START_STOP = 0x7E
ESCAPED = 0x7D
XON = 0x11
XOFF = 0x13

ADDRESS = 0X00

@dataclass()
class MOSICommand:
    command_ID: int
    data_length: int
    scaling: int
    data: bytes | None = None  # Tx data after the scaling byte, e.g. a packed float

COMMANDS = {
    "write_setpoint": MOSICommand(
        command_ID=0x00,
        data_length=5,
        scaling=0x01,
        data=None
    ),
    "read_setpoint": MOSICommand(
        command_ID=0x00,
        data_length=1,
        scaling=0x01,
        data=None
    ),
    "read_flow": MOSICommand(
        command_ID=0x08,
        data_length=1,
        scaling=0x01,
        data=None
    ),
    "read_status": MOSICommand(
        command_ID=0xD2,
        data_length=1,
        scaling=0x00,  # for 0xD2 this byte is 'clear after read': 0x00 = do not clear
        data=None
    )
}

# Execution error code from the State byte of every MISO frame (manual 6.2).
# Not to be confused with DEVICE_STATE_FLAGS below.
class ErrorCode(IntEnum):
    NO_ERROR = 0x00
    WRONG_DATA_LENGTH = 0x01
    UNKNOWN_CMD = 0x02
    ACCESS_RIGHTS = 0x03
    PARAM_OUT_OF_RANGE = 0x04
    NOT_IMPLEMENTED = 0x20
    ADDRESS_OUT_OF_RANGE = 0x21
    CHECKSUM_ERROR = 0x22
    INVALID_ADDRESS = 0x23
    WRONG_DATA_SIZE = 0x25
    FRAME_LEN_MISMATCH = 0x26
    LOOP_NOT_RUNNING = 0x2D
    HARDWARE_COMM_FAILED = 0x32
    VOLTAGE_OUT_OF_RANGE = 0x37
    GAS_PRESSURE = 0x3F
    SENSOR_BUSY = 0x42
    COMMAND_NOT_ALLOWED = 0x43
    NOT_SUPPORTED = 0x44
    FATAL_SYSTEM_ERROR = 0x7F


def error_name(code: int) -> str:
    '''Name of an execution error code, or its hex value if not in ErrorCode.'''
    try:
        return ErrorCode(code).name
    except ValueError:
        return f'error code 0x{code:02X}'


# Bits of the 32 bit device state register returned by Get Device Error
# State (0xD2), manual 6.3. Bits 11-31 are unused.
DEVICE_STATE_FLAGS = {
    0: 'boot error',
    1: 'command post processing error',
    2: 'input supply out of range',
    3: 'valve supply out of range',
    4: 'signal processor initialization failed',
    5: 'sensor communication error',
    6: 'setpoint input error',
    7: 'actuator output error',
    8: 'signal output error',
    9: 'signal buffer error',
    10: 'missing gas pressure',
}


class SensirionIO(BytesIO):
    _eol_read = bytes([START_STOP])  # readline() reads up to the next 0x7E
    # manual 4.4.2: at least 200 ms. Extra margin for the TCP serial server
    timeout = Parameter('timeout', datatype=FloatRange(0), default=0.3)

    def query(self, command: MOSICommand) -> bytes:
        '''Send one command and return the Rx Data of the reply.

        Raises HardwareError if the device reports an execution error.
        '''
        # MOSI frame content: Adr, CMD, L, Tx Data (scaling byte + optional data)
        content = bytearray([ADDRESS, command.command_ID, command.data_length, command.scaling])
        if command.data:
            content.extend(command.data)
        content.append(self.calc_crc(content))
        frame = bytes([START_STOP]) + self.escape(content) + bytes([START_STOP])
        self.log.debug('TX %s', frame.hex(' '))

        # replylen=1: communicate() reads the start byte, then calls
        # getFullReply() (below) which reads the rest up to the stop byte
        raw_response = self.communicate(frame, 1)
        self.log.debug('RX %s', raw_response.hex(' '))

        # strip start and stop first, then unescape (the CRC may be stuffed too)
        response = self.unescape(raw_response[1:-1])
        if len(response) < 5:  # Adr, CMD, State, L, CHK
            raise CommunicationFailedError(f'reply too short: {raw_response.hex(" ")}')
        if not self.check_crc(response):
            raise CommunicationFailedError(f'bad checksum: {raw_response.hex(" ")}')

        # MISO content: Adr, CMD, State, L, Rx Data, CHK
        error_code = response[2] & 0x7F  # bit 7 is the device error flag
        if error_code:
            raise HardwareError(f'{error_name(error_code)} (command 0x{command.command_ID:02X})')
        length = response[3]
        return bytes(response[4:4 + length])

    def getFullReply(self, request, replyheader):
        '''Called by BytesIO.communicate (inside its lock) after reading the
        start byte. Read the rest of the frame up to the stop byte.

        Start and stop are both 0x7E, and byte stuffing guarantees no other
        0x7E inside the frame, so the next 0x7E is always the stop byte,
        however long the stuffed frame is.
        '''
        rest = self._conn.readline(self.timeout)  # readline drops the stop byte
        return replyheader + rest + bytes([START_STOP])

    @staticmethod
    def escape(frame: bytes) -> bytes:
        '''
        Escape frame start and stop byte, escape bye, XON byte, and XOFF byte.
        Escaping happens by replacing byte by 0x7D, then the original byte with
        bit 5 inverted. e.g. 0x7E = 01111110 -> 01011110 = 0x5E

        | Original byte | Transferred data bytes | 
        | ------------- | ---------------------- |
        |       0x7E    |       0x7D, 0x5E       |
        |       0x7D    |       0x7D, 0x5D       |
        |       0x11    |       0x7D, 0x31       |
        |       0x13    |       0x7D, 0x33       |
        |_______________|________________________|

        '''
        escaped = bytearray()

        # elif: each byte must be either stuffed or copied, never both
        for b in frame:
            if b == 0x7E:
                escaped.extend((ESCAPED, START_STOP ^ 0x20))
            elif b == 0x7D:
                escaped.extend((ESCAPED, ESCAPED ^ 0X20))
            elif b == 0x11:
                escaped.extend((ESCAPED, XON ^ 0x20))
            elif b == 0x13:
                escaped.extend((ESCAPED, XOFF ^ 0x20))
            else:
                escaped.append(b)

        return bytes(escaped)

    @staticmethod
    def unescape(response: bytes) -> bytes:
        '''
        Unescape response frames. Happens by identifying the ESCAPE byte 0x7D,
        and then doing XOR 0x20 on the byte after it, and keeping that byte.

        | Original byte | Transferred data bytes | 
        | ------------- | ---------------------- |
        |       0x7E    |       0x7D, 0x5E       |
        |       0x7D    |       0x7D, 0x5D       |
        |       0x11    |       0x7D, 0x31       |
        |       0x13    |       0x7D, 0x33       |
        |_______________|________________________|

        '''
        unescaped = bytearray()
        skip_next = False
        for idx, b in enumerate(response):
            if skip_next:
                skip_next = False
                continue

            if b == ESCAPED:
                skip_next = True # skip the escape character coming after 0x7D
                if response[idx+1] == 0x5E:
                    unescaped.append(0x5E ^ 0x20)
                elif response[idx+1] == 0x5D:
                    unescaped.append(0x5D ^ 0x20)
                elif response[idx+1] == 0x31:
                    unescaped.append(0x31 ^ 0x20)
                elif response[idx+1] == 0x33:
                    unescaped.append(0x33 ^ 0x20)
                else:
                    raise CommunicationFailedError("Hex 0x7D doesn't have valid escape byte after it")
            else:
                unescaped.append(b)

        return bytes(unescaped)

    @staticmethod
    def calc_crc(frame: bytes) -> int:
        total = sum(frame)
        lsb = total & 0xFF # take the last byte of total
        checksum = ~lsb & 0xFF # invert the lsb, and only take the last byte (ignore preceding zeros)
        return checksum

    @staticmethod
    def check_crc(response: bytes) -> bool:
        '''response: unescaped Adr, CMD, State, L, Rx Data, CHK.
        The last byte is the checksum of everything before it.'''
        return response[-1] == SensirionIO.calc_crc(response[:-1])


class SFC5xxx(HasIO, Drivable):
    ioClass = SensirionIO

    target = Parameter('Target flow rate', FloatRange(unit='sccm'))
    value = Parameter('Measured flow rate', FloatRange(unit='sccm'))

    def read_value(self):
        reply = self.io.query(COMMANDS['read_flow'])
        return struct.unpack('>f', reply)[0]

    def read_status(self):
        # 0xD2 returns u32 device state register + u8 boot error (manual 5.1.3)
        reply = self.io.query(COMMANDS['read_status'])
        state_register, boot_error = struct.unpack('>IB', reply)
        if state_register == 0:
            return IDLE, ''
        active = [name for bit, name in DEVICE_STATE_FLAGS.items()
                  if state_register & (1 << bit)]
        if active == ['missing gas pressure']:
            return WARN, 'missing gas pressure, cannot reach setpoint'
        if state_register & 1:
            active.append(f'boot error code {boot_error}')
        return ERROR, ', '.join(active) or f'device state 0x{state_register:08X}'

    def read_target(self):
        reply = self.io.query(COMMANDS['read_setpoint'])
        return struct.unpack('>f', reply)[0]

    def write_target(self, target):
        # replace() makes a copy, so the shared COMMANDS entry is not modified
        cmd = replace(COMMANDS['write_setpoint'], data=struct.pack('>f', target))
        self.io.query(cmd)  # reply has no data
        return self.read_target()

    
    
