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
from frappy.core import BytesIO, HasIO, Drivable, Parameter, StringType, IntRange, FloatRange, IDLE, WARN, ERROR,Command, \
    ArrayOf, StructOf
from frappy import datatypes
from frappy.errors import CommunicationFailedError, HardwareError, RangeError
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
    scaling: int | None  # first Tx data byte (scaling or sub command), None: not sent
    data: bytes | None = None  # Tx data after the scaling byte, e.g. a packed float
    timeout: float | None = None  # for slow commands: overrides the IO timeout

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
    ),
    "read_calibration_info": MOSICommand(
        command_ID=0x40,
        data_length=5,
        scaling=0x00,
        data=None
    ),
    "read_current_calibration": MOSICommand(
        command_ID=0x44,
        data_length=1,
        scaling=0x11,
        data=None
    ),
    "load_calibration": MOSICommand(
        command_ID=0x45,
        data_length=4,
        scaling=None,  # Tx data is only the u32 location
        data=None,
        timeout=3.2  # manual 5.7.1: response time up to 1.6 s, use 2x
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

UNIT_TABLE = {
    0: "norm litre",
    1: "standard litre",
    8: "litre (liquid)",
    9: "gram",
    16: "pascal",
    17: "bar",
    18: "metre H2O",
    19: "inch H2O",
}

TIMEBASE_TABLE = {
    0: None,
    1: 'us',
    2: 'ms',
    3: 's',
    4: 'min',
    5: 'h',
    6: 'day'
}


def decode_unit(reply: bytes) -> str:
    '''Gas unit reply of 0x40/0x44 sub command 0x13 (manual 6.5):
    prefix (i8t, signed: -3 = milli), unit (u8t), timebase (u8t)'''
    prefix, unit, timebase = struct.unpack('>bBB', reply)
    return f"x10^{prefix} {UNIT_TABLE.get(unit, 'undefined')} per {TIMEBASE_TABLE.get(timebase)}"


class SensirionIO(BytesIO):
    _eol_read = bytes([START_STOP])  # readline() reads up to the next 0x7E
    # manual 4.4.2: at least 200 ms. Extra margin for the TCP serial server
    timeout = Parameter('timeout', datatype=FloatRange(0), default=0.3)

    def query(self, command: MOSICommand) -> bytes:
        '''Send one command and return the Rx Data of the reply.

        Raises HardwareError if the device reports an execution error.
        '''
        # MOSI frame content: Adr, CMD, L, Tx Data (scaling byte + optional data)
        content = bytearray([ADDRESS, command.command_ID, command.data_length])
        if command.scaling is not None:
            content.append(command.scaling)
        if command.data:
            content.extend(command.data)
        content.append(self.calc_crc(content))
        frame = bytes([START_STOP]) + self.escape(content) + bytes([START_STOP])
        self.log.debug('TX %s', frame.hex(' '))

        # Slow commands (e.g. Load Calibration, up to 1.6 s) bring their own
        # timeout. Holding the lock means no other request runs with it.
        # (_lock is an RLock, so communicate() can take it again inside.)
        with self._lock:
            normal_timeout = self.timeout
            if command.timeout:
                self.timeout = command.timeout
            try:
                # replylen=1: communicate() reads the start byte, then calls
                # getFullReply() (below) which reads the rest up to the stop byte
                raw_response = self.communicate(frame, 1)
            finally:
                self.timeout = normal_timeout
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

    target = Parameter('Target flow rate', FloatRange(unit='slm'))
    value = Parameter('Measured flow rate', FloatRange(unit='slm'))
    gas = Parameter('Current calibrated gas loaded',StringType(), readonly=True)
    gas_ID = Parameter('Current calibrated gas ID',IntRange(0, 0xFFFFFFFF), readonly=True)  # u32
    units = Parameter('Units for current configuration', StringType(), readonly=True)
    calib_mem = Parameter('Size of calibration memory', IntRange(0,200), readonly=True)
    conductivity = Parameter('Thermal conductivity reference value of the loaded gas (raw sensor value)',
                             IntRange(0, 0xFFFF), readonly=True)
    # No read_calibrations method: this is not polled. It is filled at
    # startup (initialReads) and by the refresh_calibrations command.
    calibrations = Parameter('Valid calibrations in the device memory, load with load_calibration(location)',
                             ArrayOf(StructOf(location=IntRange(0, 255),
                                              gas=StringType(),
                                              gas_ID=IntRange(0, 0xFFFFFFFF),
                                              fullscale=FloatRange(),
                                              unit=StringType()), 0, 255),
                             readonly=True)

    def initialReads(self):
        # Called once from the poll thread at startup. callPollFunc logs an
        # error instead of raising it, so a failure here can't stop the poll
        # thread. Call refresh_calibrations later if the device was offline.
        self.callPollFunc(self.refresh_calibrations)

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

    def read_gas(self):
        gas_descr = self.io.query(COMMANDS['read_current_calibration'])
        return str(gas_descr.rstrip(b'\x00').decode('utf-8'))

    def read_gas_ID(self):
        cmd = replace(COMMANDS['read_current_calibration'])
        cmd.scaling = 0x12
        gas_ID = int.from_bytes(self.io.query(cmd), byteorder='big')
        return gas_ID

    def read_units(self):
        cmd = replace(COMMANDS['read_current_calibration'])
        cmd.scaling=0x13
        return decode_unit(self.io.query(cmd))

    def read_conductivity(self):
        # 0x44 sub command 0x17 (firmware >= 1.41): u16 reference value
        cmd = replace(COMMANDS['read_current_calibration'], scaling=0x17)
        return struct.unpack('>H', self.io.query(cmd))[0]

    def read_calib_mem(self):
        cmd = MOSICommand(
            command_ID=0x40,
            data_length=1,
            scaling=0x00
        )
        reply = int.from_bytes(self.io.query(cmd), byteorder='big')
        return reply

    def write_target(self, target):
        # replace() makes a copy, so the shared COMMANDS entry is not modified
        cmd = replace(COMMANDS['write_setpoint'], data=struct.pack('>f', target))
        self.io.query(cmd)  # reply has no data
        return self.read_target()

    def _calibration_info(self, info_type, location):
        '''Get Calibration Information (0x40) about one memory location.
        info_type: 0x10 valid, 0x11 gas, 0x12 gas ID, 0x13 unit, 0x14 fullscale'''
        cmd = replace(COMMANDS['read_calibration_info'], scaling=info_type,
                      data=struct.pack('>I', location))
        return self.io.query(cmd)

    @Command()
    def refresh_calibrations(self):
        '''Re-read the list of valid calibrations from the device memory.'''
        calibrations = []
        # not every location holds a valid calibration, and the invalid ones
        # are not necessarily at the end (manual 5.7)
        for location in range(self.read_calib_mem()):
            if not self._calibration_info(0x10, location)[0]:
                continue
            calibrations.append({
                'location': location,
                'gas': self._calibration_info(0x11, location).rstrip(b'\x00').decode('utf-8'),
                'gas_ID': int.from_bytes(self._calibration_info(0x12, location), byteorder='big'),
                'fullscale': struct.unpack('>f', self._calibration_info(0x14, location))[0],
                'unit': decode_unit(self._calibration_info(0x13, location)),
            })
        self.calibrations = calibrations
        self.log.info('found %d valid calibrations', len(calibrations))

    @Command(IntRange(0, 255))
    def load_calibration(self, location):
        '''Load the calibration at <location> (see calibrations) and run the controller.

        Writes the EEPROM (limited write cycles, manual 5.7.1), so use it to
        change gas, never periodically. Reloading the current one is harmless.
        '''
        valid = [c['location'] for c in self.calibrations]
        if location not in valid:
            raise RangeError(f'no valid calibration at location {location} (valid: {valid}).'
                             ' Run refresh_calibrations if the list is empty')
        cmd = replace(COMMANDS['load_calibration'], data=struct.pack('>I', location))
        self.io.query(cmd)  # reply has no data
        # update the parameters that describe the loaded calibration
        self.read_gas()
        self.read_gas_ID()
        self.read_units()
        self.read_conductivity()
        self.read_target()