# *****************************************************************************
#
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
#   Markus Zolliker <markus.zolliker@psi.ch>
#
# *****************************************************************************

import time
from math import copysign
from frappy.datatypes import BoolType, FloatRange
from frappy.core import Parameter, BUSY
from frappy.lib import merge_status, clamp
from frappy.errors import RangeError


class HasRamp:
    """software ramp"""
    # make sure it is a drivable
    status = Parameter()
    target = Parameter()
    ramp = Parameter('ramp rate', FloatRange(0, unit='$/min'), default=0, readonly=False)
    ramp_used = Parameter('False: infinite ramp', BoolType(), default=False, readonly=False)
    setpoint = Parameter('ramping setpoint', FloatRange(unit='$'), readonly=True)
    maxdif = Parameter('''max. difference between setpoint and value
    
                        stop ramp then value lags behind setpoint by more than maxdif
                        maxdif=0: use 'ramp' value (value lags 1 minute behind setpoint)
                        ''',
                       FloatRange(0, unit='$'), default=0, readonly=False)
    rampinterval = Parameter('interval for changing the setpoint', FloatRange(0, unit='s'),
                             default=1, readonly=False)
    workingramp = Parameter('effective ramp', FloatRange(unit='$/min'))

    _ramp_status = None
    _last_point = None
    _buffer = 0

    def doPoll(self):
        super().doPoll()  # suppose that this is reading value and status
        self.ramp_step(self.target)

    def ramp_step(self, target):
        now = time.time()
        setpoint = self.setpoint
        if self._ramp_status is not None:
            if setpoint == target:
                self._ramp_status = None  # at target
                self.workingramp = 0
            elif self._last_point:
                sign = copysign(1, target - setpoint)
                prev_t, prev_v = self._last_point
                if self.value == prev_v:
                    return  # no reads happened
                delay = (now - prev_t) / 60.0  # minutes !
                slope = (self.value - prev_v) / max(1e-5, delay)
                dif = (setpoint - self.value) * sign
                maxdif = self.maxdif or self.ramp
                if dif < maxdif:
                    # reduce ramp when slope is bigger than ramp
                    ramp = max(2 * self.ramp - sign * slope, 0) + self._buffer
                    if ramp > self.ramp:
                        self._buffer = min(ramp - self.ramp, self.ramp)
                        ramp = self.ramp
                    else:
                        self._buffer = 0
                    setpoint += sign * delay * ramp
                    if sign * (setpoint - target) >= 0:
                        self.write_setpoint(target)  # do not overshoot
                        self.workingramp = 0
                        self._ramp_status = None  # at target
                    else:
                        if ramp != self.workingramp:
                            self.workingramp = sign * ramp
                        self.write_setpoint(setpoint)
                        self._ramp_status = 'ramping'
                else:
                    self._ramp_status = 'holding'
        self._last_point = now, self.value
        self.read_status()

    def read_status(self):
        status = super().read_status()
        if self._ramp_status is None:
            if self.pollInfo.fast_flag:
                self.setFastPoll(False)
            return status
        if self.pollInfo.interval != self.rampinterval:
            self.setFastPoll(True, self.rampinterval)
        return merge_status((BUSY, self._ramp_status), status)

    def write_ramp(self, ramp):
        if ramp:
            self.write_ramp_used(True)
        else:
            raise RangeError('ramp must not 0, use ramp_used = False to disable ramping')
        return ramp

    def write_ramp_used(self, used):
        if used != self.ramp_used:
            self.ramp_used = used
            if self._ramp_status:
                self.write_target(self.target)

    def write_setpoint(self, setpoint):
        super().write_target(setpoint)
        return setpoint

    def read_target(self):
        if not self._ramp_status:
            return super().read_target()
        return self.target

    def write_target(self, target):
        if self.ramp_used:
            if self.parameters['setpoint'].readerror:
                self.write_setpoint(self.read_value())
            self._ramp_status = 'changed target'
            self._last_point = time.time(), self.value  # reference for the first ramp step
            self.setFastPoll(True, self.rampinterval)
            self.ramp_step(target)
            return target
        self._ramp_status = None
        self.write_setpoint(target)
        return target


class SimpleRamp(HasRamp):
    """simple time based software ramp

    Like HasRamp, but the setpoint advances by ramp * elapsed time on every
    step, regardless of whether the value follows: no waiting when the value
    lags behind, no slope correction, and steps are taken even when the value
    did not change.

    usage: put it as the first base class, and provide write_setpoint() for
    sending the setpoint to the hardware (default: super().write_target)
    """
    maxdif = None  # not used: the ramp does not wait for the value

    _last_ramp_time = None

    def write_target(self, target):
        # time reference for the ramp: the first step happens on the next poll
        self._last_ramp_time = time.time()
        return super().write_target(target)

    def ramp_step(self, target):
        now = time.time()
        if self._ramp_status is not None:
            setpoint = self.setpoint
            if setpoint == target:
                self._ramp_status = None  # at target
                self.workingramp = 0
            else:
                sign = copysign(1, target - setpoint)
                delay = (now - (self._last_ramp_time or now)) / 60.0  # minutes !
                setpoint += sign * self.ramp * delay
                if sign * (setpoint - target) >= 0:
                    self.write_setpoint(target)  # do not overshoot
                    self.workingramp = 0
                    self._ramp_status = None  # at target
                else:
                    self.write_setpoint(setpoint)
                    self.workingramp = sign * self.ramp
                    self._ramp_status = 'ramping'
        self._last_ramp_time = now
        self.read_setpoint()
        self.read_status()
