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
''' This is a driver for a Cryomech CP1000 helium compressor, that  uses hex binary to access a data dictionary'''

from frappy.core import BytesIO, HasIO, FloatRange, StringType, \
    IDLE, WARN, ERROR, Readable, Parameter, Property, Communicator
from frappy.errors import CommunicationFailedError
import struct

CMD = 0x80
START = 0x02
ADDRESS = 0x10
CR = 0x0D
READ = 0X63

data_dictionary = {
    "CPU_temp" : bytes([ADDRESS, CMD, READ, 0x45, 0x4C, 0x00]),
    "Compressor_current" : bytes([ADDRESS, CMD, READ, 0x63, 0x8B, 0x00]),
    "Compressor_minutes" : bytes([ADDRESS, CMD, READ, 0x45, 0x4C, 0x00]),
    "Input_water_temp" : bytes([ADDRESS, CMD, READ, 0x0D, 0x8F, 0x00]),
    "Output_water_temp": bytes([ADDRESS, CMD, READ, 0x0D, 0x8F, 0x01]),
    "Error_code_status": bytes([ADDRESS, CMD, READ, 0x65, 0xA4, 0x00])
}    

# TODO: check whether EOL is added to bytes sent
class CryomechIO(BytesIO):
    """
    IO class for the Cryomech CP1000 helium compressor. The protocol is Sycon Multi Drop Protocol 
    (SMDP) which is a binary protocol. The structure of a packet is as follows:

    <STX><Address><Command-Response>[<Data>...]<Checksum1><Checksum2><CR>

    <STX>: Start of Text> is a single byte with value 0x02. This syncs the receiver up to receive 
            a new message and purges data before it.

    <Address>: The address field, which is a single byte. Valid ranges are 10 hex to FE hex (16 
            to 254 decimal). The address is used to identify the device on the bus. The CP1000 
            has a default address of 0x10 (16 decimal) if using RS232.

    <Command-Response>: The command or response field, which is a single byte. The left-hand 
            4 bits of the byte are the command (highest binary values). The RSPF bit is zero when 
            going from master to slave, and RSP fields are zero. When going from slave to master, 
            the CMD bits are identical to the command sent, but the RSP field will be non-zero, 
            and the response fields will contain the response data. The byte packet is indicated below.

                                        [<CMD3><CMD2><CMD1><CMD0><RSPF><RSP2><RSP1><RSP0>]

            ** IMPORTANT ** For PS1000, the <Command-Response> byte is 0x80

    <Data>: The data field, which is a variable length field. The data field can be 0 to 255 bytes long. 
            The data field is used to send or receive data from the device. To read data, this section 
            needs to have the following format:

            <Data> = <'c'><hashval-int><array index-byte>

                <'c'> is 0x63
                <hashval-int> is the hash value of the data item to read, which is a 2 byte integer. 
                    The hash value is obtained from the data dictionary.
                <array index byte> is a 1-byte index for the data object (for arrays). If response is 
                integer, this byte is ignored 

    <CKSUM1> and <CKSUM2>: The checksum fields, which are each a single byte. The checksum is
            calculated by summing all the bytes in the message, excluding the STX and CR bytes
            and taking the least significant byte of the sum. The checksum is used to verify that
            the message was received correctly. The checksum is calculated as follows:

    <CR>: Carriage Return 0x0D

    Example: To read Compressor current, the command would be:
                0x02 | 0x10 | 0x80 | 0x63 | 0x63 | 0x8B | 0x3E | 0x31 | 0x0D       

    """
    #identification = [('ADDRESS,CMD, READ, 0x2B, 0x0D, 0x00]), b'59085')]
    #default_settings = {"baudrate" : 9600, "bytesize" : 8, "timeout" : 2, "stopbits": 1, "parity": 'NONE'}
    _eol_read = b'\x0D'

    def communicate(self, command):
        #with self._lock:
        frame = bytearray()
        frame.append(START)
        #frame.append(ADDRESS)
        #frame.append(CMD)
        frame.extend(self.add_escape_chars(command)) # Add escape bytes to command and append to  frame

        checksum = self.calculate_crc(command) # calculate checksum on <ADDRESS>, <CMD>, <Data bytes> 
        #self.log.info(f"checksum argument = {command}")
        #self.log.info(f"checksum output = {checksum}")
        frame.extend(checksum)
        frame.append(CR)
        #self.log.info(f"byte frame size = {len(frame)}")
        #self.log.info(f"byte frame after escape chars: {frame}")
        self.log.info(f"frame = {frame}")


        raw_response = super().communicate(frame, 14)
        self.log.info(f"raw response: {raw_response}")
        unescaped = self.remove_escape_chars(raw_response[1:-3])
        response = bytearray()
        response.append(raw_response[0])
        response.extend(unescaped)
        response.extend(raw_response[-3:])

        self.log.info(f"unescaped response = {response}")
        if response[0] != 0x02:
            raise CommunicationFailedError('Invalid response: missing STX')
        if response[1] != ADDRESS:
            raise CommunicationFailedError('Invalid response: incorrect address')

        if self.check_crc(response) == False:
            self.log.info(f"response for checksum = {response[1:-3]}")
            raise CommunicationFailedError(f'Bad crc: response {response[-3:-1]} does not equal {self.calculate_crc(bytes(response[1:-3]))}')
        #if response[2] != 0x80:
        #    raise CommunicationFailedError('Invalid response: incorrect command/response byte')
        return response[7:-3]  # Return the data field, excluding STX,

    @staticmethod
    def calculate_crc(command: bytes) -> bytes:
        checksum = sum(command) % 256 # Checksum is ADDRESS + CMD + COMMAND modulo 256
        
        cksum1 = 0x30 + ((checksum & 0xF0) >> 4) # first byte of checksum is the most significant 4 bits of the checksum (pushed to the right) plus 0x30 
        cksum2 = 0x30 + (checksum & 0x0F) # second byte of checksum isthe least significant 4 bits of the checksum plus 0x30
        return bytes([cksum1, cksum2])

    @staticmethod
    def check_crc(response: bytearray) -> bool:
        if bytes(response[-3:-1]) != CryomechIO.calculate_crc(response[1:-3]):
            return False
        else:
            return True

    @staticmethod
    def add_escape_chars(frame: bytes) -> bytes:
        """
        Apply protocol escaping.

        0x02 -> 0x07 0x30
        0x0D -> 0x07 0x31
        0x07 -> 0x07 0x32
        """
        escaped = bytearray()

        for b in frame:
            if b == 0x02:
                escaped.extend((0x07, 0x30))
            elif b == 0x0D:
                escaped.extend((0x07, 0x31))
            elif b == 0x07:
                escaped.extend((0x07, 0x32))
            else:
                escaped.append(b)

        return bytes(escaped)

    @staticmethod
    def remove_escape_chars(response: bytes) -> bytes:
        """
        Remove protocol escaping.

        0x07 0x30 -> 0x02 
        0x07 0x31 -> 0x0D
        0x07 0x32 -> 0x07
        """

        unescaped = bytearray()
        skip_next = False
        for idx, b in enumerate(response):
            if skip_next:
                skip_next = False
                continue

            if b == 0x07:
                skip_next = True # skip the escape character coming after 0x07
                if response[idx+1] == 0x30:
                    unescaped.append(0x02)
                elif response[idx+1] == 0x31:
                    unescaped.append(0x0D)
                elif response[idx+1] == 0x32:
                    unescaped.append(0x07)
                else:
                    raise CommunicationFailedError("Hex 0x07 doesn't have valid escape byte after it")
            else:
                unescaped.append(b)

        return bytes(unescaped)


class CP1000(HasIO, Readable):
    ioClass = CryomechIO
    value = Parameter('Input water temperature', datatype=FloatRange(unit='C'))
    #CPU_temp= Parameter('CPU Temperature', datatype=FloatRange(unit='C'))

    def read_value(self):
        response = self.communicate(data_dictionary["Output_water_temp"])
        float_val = float(struct.unpack('>i', response)[0]/10)
        return float_val

    def read_status(self):
        response = int(struct.unpack('>i', self.communicate(data_dictionary['Error_code_status']))[0])

        if response == 0:
            text = "no error"
            return IDLE, text
        elif response == 2:
            text = "5V power too high"
        elif response == 3:
            text = "5V power too low"
        elif response == 4:
            text = "Compressor is in lockout mode"
        else:
            text = "no error"
        return ERROR, text

    #def read_CPU_temp(self):
    #    return float(self.communicate(data_dictionary["CPU_temp"]))
        
