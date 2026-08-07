#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from frappy.core import Readable, Writable, Parameter, Communicator, Module, FloatRange, HasIO, StringIO, Property, StringType, \
    IDLE, BUSY, WARN, ERROR, DISABLED, Drivable, IntRange, BoolType
from frappy.lib import parse_host_port
from frappy.errors import CommunicationFailedError
from mecom import MeComTcp
import time


class MecomProtocol(StringIO):
    """NOT IMPLEMENTED YET: Protocol for MeCom communication with the TEC controller."""

    end_of_line = '\r'

    @staticmethod
    def crc16_algorithm(crc: int, ch: int) -> int:
        """
        Update CRC16-CCITT (XMODEM) with one byte.

        Parameters
        ----------
        crc : int
            Current CRC value.
        ch : int
            Byte value (0-255).

        Returns
        -------
        int
            Updated CRC value.
        """

        gen_poly = 0x1021

        crc ^= (ch & 0xFF) << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ gen_poly
            else:
                crc <<= 1

        return crc & 0xFFFF



class MeComIO(Communicator):
    uri = Property("""uri for serial connection

                   one of the following:

                   - ``tcp://<host address>:<portnumber>`` (see :class:`frappy.lib.asynconn.AsynTcp`)

                   - ``serial://<serial device>?baudrate=<value>...`` (see :class:`frappy.lib.asynconn.AsynSerial`)
                   """, datatype=StringType())
    
    identification = [('100', '1161')]  # MeCom sends '1161' on connect, and the reply is checked to match the regexp '1161'
    default_settings = {'port': 50000, 'baudrate': 57600}
    min_request_interval = 0.05  # MeCom requires a wait time of 50 ms between commands

    def initModule(self):
        super().initModule()
        if self.uri.startswith('tcp://'):
            uri = self.uri[6:]
        host, port = parse_host_port(uri)
        self.mc = MeComTcp(
            ipaddress = host,
            ipport = port,
        )

        self._last_request = 0

    def _wait(self):
        now = time.monotonic()
        elapsed = now - self._last_request

        if elapsed < self.min_request_interval:
            time.sleep(self.min_request_interval - elapsed)

        self._last_request = time.monotonic()

    def checkHWIdent(self):
        if not self.identification:
            return
        idents = iter(self.identification)
        command, regexp = next(idents)
        reply = self.communicate(command)
        if not regexp == reply:
            if self.retry_first_idn:
                self.log.debug('first ident command not successful.'
                               ' retrying in case of garbage data.')
                idents = iter(self.identification)
            else:
                self.closeConnection()
                raise CommunicationFailedError(f'bad response: {reply!r}'
                                               f' does not match {regexp!r}')
        for command, regexp in idents:
            reply = self.communicate(command)
            if not regexp == reply:
                self.closeConnection()
                raise CommunicationFailedError(f'bad response: {reply!r}'
                                               f' does not match {regexp!r}')

    def communicate(self, parameter, address, channel):
        self._wait()
        return self.mc.get_parameter(
            parameter_id=parameter,
            address=address,
            parameter_instance=channel
        )

    def set(self, parameter, address, channel, value):
        self._wait()
        self.mc.set_parameter(
            parameter_id=parameter,
            address=address,
            parameter_instance=channel,
            value=value
        )


class TECBase(HasIO, Readable):
    ioClass = MeComIO
    address = Parameter('TEC address', datatype=IntRange(3, 255), default=4)
    channel = Parameter('TEC channel', datatype=IntRange(0, 1), default=0)
    value = Parameter('Hardware Version', datatype=StringType(), readonly=True)
    
    def read_value(self):
        return self.io.communicate(101, self.address, self.channel)

    
class TECDrivable(Drivable, TECBase):
    pass


class SamplePosition(TECDrivable):
    value = Parameter('Sample temperature', datatype=FloatRange(-100, 100, unit='C'))
    target = Parameter('Target temperature', datatype=FloatRange(-100, 100, unit='C'))
    setpoint = Parameter('Temperature setpoint', datatype=FloatRange(-100, 100, unit='C'))
    output_enabled = Parameter('Output enabled?', datatype=BoolType(), default=False)
    tolerance = Parameter('convergence criterion', FloatRange(0), default=0.1, readonly=False)
    _driving = False

    def read_value(self):
        return self.io.communicate(1000, self.address, self.channel)

    def read_setpoint(self):
        return self.io.communicate(3000, self.address, self.channel)

    def read_output_enabled(self):
        return bool(self.io.communicate(2010, self.address, self.channel))
    
    def write_target(self, value):
        #self.enable_loop(self.address, self.channel)
        self.io.set(3000, self.address, self.channel, value)
        self.status = BUSY, 'target changed'
        return value

    def write_output_enabled(self, value):
        self.io.set(2010, self.address, self.channel, int(value))
        return value

    def read_status(self):
        code = int(self.io.communicate(1200, self.address, self.channel))
        if code == 0:
            text = 'Temperature regulation is not active'
            return WARN, text
        elif code == 1:
            text = 'temperature out of tolerance'
        elif code == 2:
            text = 'temperature within tolerance'
        
        if abs(self.target - self.value) > self.tolerance:
            if self._driving:
                return BUSY, 'approaching setpoint'
            return WARN, 'temperature out of tolerance'
        else:  # within tolerance: simple convergence criterion
            self._driving = False
            return IDLE, ''

'''
class SampleTemperature(TECBase):
    value = Parameter('Sample temperature', datatype=FloatRange(-100, 100, unit='C'))


    def read_value(self):
        return self.io.communicate(1000, self.address, self.channel)

    def read_status(self):
        code = int(self.io.communicate(1200, self.address, self.channel))
        if code == 0:
            text = 'Temperature regulation is not active'
            return WARN, text
        elif code == 1:
            text = 'temperature out of tolerance'
            return BUSY, text
        elif code == 2:
            text = 'temperature within tolerance'
        return IDLE, text


class TemperatureSetpoint(TECBase):
    ioClass = MeComIO
    target = Parameter('Target temperature', datatype=FloatRange(-100, 100, unit='C'))

    def read_target(self):
        return self.io.communicate(3000, self.address, self.channel)


class TemperatureLoop(SampleTemperature, TemperatureSetpoint, Drivable):
    ioClass = MeComIO
    heater_range = Parameter('Heater power range', datatype=IntRange(0, 5), default=3)
    tolerance = Parameter('Convergence criterion', datatype=FloatRange(0), default=0.1, readonly=False)
    _driving = False

    def write_target(self, target):
        # reactivate heater in case it was switched off
        self.io.set(parameter=2010, address=self.address, channel=self.channel, value=1)
        self.io.set(parameter=3000, address=self.address, channel=self.channel, value=target)
        self._driving = True
        self.status = BUSY, 'target changed'
        return target

    def read_status(self):
        code = int(self.io.communicate(1200, self.address, self.channel))
        if code == 0:
            text = 'Temperature regulation is not active'
            return WARN, text
        elif code == 1:
            text = 'temperature out of tolerance'
            return BUSY, text
        elif code == 2:
            text = 'temperature within tolerance'
        
        if abs(self.target - self.value) > self.tolerance:
            if self._driving:
                return BUSY, 'approaching setpoint'
            return WARN, 'temperature out of tolerance'
        else:  # within tolerance: simple convergence criterion
            self._driving = False
            return IDLE, ''

class TECOutput(TECBase):
    ioClass = MeComIO
    value = Parameter('Output enabled?', datatype=BoolType())

    def enable_loop(self, address, channel):
        self.io.set(parameter=2010, address=address, channel=channel, value=1)

#    def read_target(self):
#        return self.io.communicate(3000, self.address, self.channel)

'''
