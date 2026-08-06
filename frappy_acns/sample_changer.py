#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from frappy.core import Readable, Writable, Parameter, Communicator, Module, FloatRange, HasIO, StringIO, Property, StringType, \
    IDLE, BUSY, WARN, ERROR, DISABLED, Drivable, IntRange, BytesIO
from frappy.lib import parse_host_port
from mecom import MeComTcp



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



class MeComIO(StringIO):
    uri = Property("""uri for serial connection

                   one of the following:

                   - ``tcp://<host address>:<portnumber>`` (see :class:`frappy.lib.asynconn.AsynTcp`)

                   - ``serial://<serial device>?baudrate=<value>...`` (see :class:`frappy.lib.asynconn.AsynSerial`)
                   """, datatype=StringType())
    
    identification = [('*IDN?', 'LSCI,.*')]
    default_settings = {'port': 50000, 'baudrate': 57600}
    wait_before = 0.5  # MeCom requires a wait time of 50 ms between commands

    def initModule(self):
        super().initModule()
        if self.uri.startswith('tcp://'):
            uri = self.uri[6:]
        host, port = parse_host_port(uri)
        self.mc = MeComTcp(
            ipaddress = host,
            ipport = port,
        )

    def communicate(self, parameter, address, channel):
        return self.mc.get_parameter(
            parameter_id=parameter,
            address=address,
            parameter_instance=channel
        )

    def set(self, parameter, address, channel, value):
        self.mc.set_parameter(
            parameter_id=parameter,
            address=address,
            parameter_instance=channel,
            value=value
        )

class ObjectTemperature(HasIO, Readable):
    ioClass = MeComIO
    address = Parameter('TEC address', datatype=IntRange(3, 255), default=4)
    channel = Parameter('TEC channel', datatype=IntRange(0, 1), default=0)
    value = Parameter('Object temperature', datatype=FloatRange(-100, 100, unit='C'))

    def read_value(self):
        return self.io.communicate(1000, self.address, self.channel)

    def read_status(self):
        code = int(self.io.communicate(1200, self.address, self.channel))
        if code == 0:
            text = 'Temperature regulation is not active'
            return DISABLED, text
        elif code == 1:
            text = 'temperature out of tolerance'
            return BUSY, text
        elif code == 2:
            text = 'temperature within tolerance'
            return IDLE, text

class TECOutput(HasIO, Writable):
    ioClass = MeComIO



class TEC1161Base(HasIO, Drivable):
    ioClass = MeComIO
    address = Parameter('TEC address', datatype=IntRange(3, 255), default=4)
    channel = Parameter('TEC channel', datatype=IntRange(0, 1), default=0)
    value = Parameter('TEC value', datatype=FloatRange(-100, 100, unit='C'))
    target = Parameter('TEC target', datatype=FloatRange(-100, 100, unit='C'))
    
    def read_value(self):

        return self.io.communicate(1000, self.address, self.channel)

    def read_target(self):
        return self.io.communicate(3000, self.address, self.channel)

    
    def enable_loop(self, address, channel):
        self.io.set(parameter=2010, address=address, channel=channel, value=1)

    def write_target(self, value):
        self.enable_loop(self.address, self.channel)
        self.io.set(3000, self.address, self.channel, value)

        return value

