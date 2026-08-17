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
from frappy.core import StringIO, HasIO, IntRange, StringType, \
    IDLE, WARN, ERROR, Readable, Parameter, Property
from frappy.errors import CommunicationFailedError

class CryoMagneticsIO(StringIO):
    end_of_line = '\r'
    default_settings = {'baudrate': 9600, 'parity': 'N', 'bytesize': 8, 'stopbits': 1}
    identification = [('*IDN?', 'Cryomagnetics,LM*.*')]


    def communicate(self, command):
        with self._lock:
            response = super().communicate(command)
            return response
            

class HeLevel(HasIO, Readable):
    ioClass = CryoMagneticsIO
    value = Parameter(unit='cm')
    channel = Property('Channel', IntRange(1, 2), default=1)
    

    def read_value(self):
        response = self.io.communicate(f'MEAS? {self.channel}')

        try:
            value, units = response.split(' ')    
            return float(value)
        except: 
            raise CommunicationFailedError(f'Unexpected response: {response}')
    # 

    def read_status(self):
        response = self.io.communicate(f'STAT?')
        if self.channel == 1:
            response = int(response.split(',')[0])
        else:
            response = int(response.split(',')[1])

        status_dict = {
            "read in progress" : bool(response & (1 << 0)),
            "control (refill) active" : bool(response & (1 << 1)),
            "ctrl timeout occurred" : bool(response & (1 << 2)),
            "ctrl refill inhibited by mode OFF or timeout" : bool(response & (1 << 3)),
            "alarm limit exceeded" : bool(response & (1 << 4)),
            "open sensor detected" : bool(response & (1 << 5)),
            "burnout condition was detected" : bool(response & (1 << 6)),
        }


        if any(status_dict.values()):
            active = [name for name, value in status_dict.items() if value]
            return (WARN, f'Active status flags: {", ".join(active)}')
        else:
            return (IDLE, 'no error')
        
