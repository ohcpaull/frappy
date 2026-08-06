#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 29 09:24:07 2024
@author: andreaplank
"""
from frappy.core import Readable, Parameter, FloatRange, BoolType, StringIO, HasIO, \
    Property, StringType, Drivable, IntRange, IDLE, BUSY, ERROR, nopoll
from frappy.errors import CommunicationFailedError


class PfeifferProtocol(StringIO): 
    end_of_line = '\r'

    @staticmethod
    def calculate_crc(data):
        crc = sum(ord(ch) for ch in data) % 256
        return f'{crc:03d}'

    @staticmethod
    def check_crc(data):
        if data[-3:] != PfeifferProtocol.calculate_crc(data[:-3]):
            raise CommunicationFailedError('Bad crc')

    def communicate(self, cmd):
        cmd += self.calculate_crc(cmd) 
        reply = super().communicate(cmd)
        self.check_crc(reply)
        return reply[:-3]


class PfeifferMixin(HasIO): 
    ioClass = PfeifferProtocol    
    address= Property('Addresse', datatype= IntRange())

    def data_request_u_expo_new(self, parameter_nr):
        reply = self.communicate(f'{self.address:03d}00{parameter_nr:03d}02=?')
        
        assert int(reply[5:8]) == parameter_nr
        
        assert int(reply[0:3]) == self.address
        
        try:
            exponent = int(reply[14:16])-23
        except ValueError:
            raise CommunicationFailedError(f'got {reply[10:16]}')

        return float(f'{reply[10:14]}e{exponent}')
        
    def data_request_old_boolean(self, parameter_nr):

        reply = self.communicate(f'{self.address:03d}00{parameter_nr:03d}02=?')
        assert int(reply[5:8]) == parameter_nr, f"Parameter number mismatch: expected {parameter_nr}, got {int(reply[5:8])}"
        
        assert int(reply[0:3]) == self.address, f"Address mismatch: expected {self.address}, got {int(reply[0:3])}"

        if reply[12] == "1": 
                value = True
        elif reply[12] == "0": 
                value = False
        else:
                raise CommunicationFailedError(f'got {reply[10:16]}')    
                
        return value
    
    def data_request_u_real(self, parameter_nr):
        reply = self.communicate(f'{self.address:03d}00{parameter_nr:03d}02=?')

        assert int(reply[5:8]) == parameter_nr
        
        assert int(reply[0:3]) == self.address
        
        try:
            value = float(reply[10:16])/100
        except ValueError:
            raise CommunicationFailedError(f'got {reply[10:16]}')
        
        return value

    def data_request_u_int(self, parameter_nr):
        reply = self.communicate(f'{self.address:03d}00{parameter_nr:03d}02=?')

        if reply[8] == "0":
            reply_length = int(reply[9])
        else:
            reply_length = int(reply[8:10])
                
        try:
            if reply[10 : 10 + reply_length] == "000000": 
                value = 0
            else: 
                value = float(reply[10 : 10 + reply_length].lstrip("0"))
        except ValueError:
            raise CommunicationFailedError(f'got {reply[10:16]}')    
            
        return value
    
    def data_request_string(self, parameter_nr):
        reply = self.communicate(f'{self.address:03d}00{parameter_nr:03d}02=?')

        assert int(reply[5:8]) == parameter_nr
        
        assert int(reply[0:3]) == self.address
        
        return str(reply[10:16])

    def control_old_boolean(self, parameter_nr, target):
        if target: 
            val = 1
        else:
            val = 0

        cmd = f'{self.address:03d}10{parameter_nr:03d}06{str(val)*6}'

        reply = self.communicate(cmd)

        assert cmd == reply, f'got {reply}  instead of {cmd} '
        
        try:
            if reply[11] == "1":
                value = 1
            else:
                value = 0
        except ValueError:

            raise CommunicationFailedError(f'got {reply[10:16]}')
        
        return value


class RPT200(PfeifferMixin, Readable):   
    value = Parameter('Pressure', FloatRange(unit='mbar'))

    def read_value(self):
        return self.data_request_u_expo_new(740)

    def read_status(self):
        errtxt = self.data_request_string(303)
        if errtxt == "000000":
            return IDLE, ''
        else: 
            return ERROR, errtxt

class CPT200(PfeifferMixin, Readable):
    value= Parameter('Pressure', FloatRange(unit='mbar'))

    def read_value(self):
        return self.data_request_u_expo_new(740)

    def read_status(self):
        errtxt = self.data_request_string(303)
        if errtxt == "000000":
            return IDLE, ''
        else: 
            return ERROR, errtxt

class TCP400(PfeifferMixin, Drivable, Readable):
    speed= Parameter('Rotational speed', FloatRange(unit='Hz'), readonly = False)
    target= Parameter('Pumping station', BoolType())
    current= Parameter('Current consumption', FloatRange(unit='%'))
    value = Parameter('Turbopump state', BoolType())    
    temp = Parameter('temp', FloatRange(unit='C'))

    def read_temp (self):
        return self.data_request_u_int(326)
   
    def read_speed(self):
        return self.data_request_u_int(309)

    def read_value(self):
        return self.data_request_old_boolean(10) 

    def read_current(self):
        return self.data_request_u_real(310)
    
    def write_target(self, target): 
        return self.control_old_boolean(10, target)
             
    def read_target(self):
        return self.data_request_old_boolean(10)
        
    def read_status(self): 
        if not self.data_request_old_boolean(306):
            if self.target:
                return BUSY, 'ramping up'
            else:
                return IDLE, ''
        else: 
            return IDLE, 'at targetspeed'


class Omnicontrol(PfeifferMixin, Drivable, Readable):
    value = Parameter('Valve state', BoolType())
    target = Parameter('Valve state', BoolType(), readonly=False)

    def read_value(self):
        return self.data_request_old_boolean(10)

    def write_target(self, target):
        return self.control_old_boolean(10, target)

    def read_target(self):
        return self.data_request_old_boolean(10)

    def read_status(self):
        if self.read_value() == self.read_target():
            return IDLE, ''
        else:
            return BUSY, 'changing valve state'