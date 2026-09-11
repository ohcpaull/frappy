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
"""Protocol-level simulator for a LakeShore Model 336 temperature controller.

This is a :class:`frappy.modules.Communicator`, so it is used *instead of* the
real ``IO('io', 'tcp://...')`` line in a config file: the real driver classes in
``frappy_acns.lakeshore`` (``TemperatureSensor``, ``TemperatureLoop``,
``Heater``) run completely unchanged and send their normal LS336 command strings
here.

Only the subset of the LS336 command set that ``frappy_acns.lakeshore`` actually
uses is implemented:

    ``*IDN?``                     identification
    ``*OPC?``                     operation complete -> ``1``
    ``KRDG?<ch>``                 kelvin reading of input channel A..D
    ``RDGST?<ch>``                reading status bitfield  (``0`` healthy,
                                 ``160`` for a disconnected input)
    ``SETP?<n>`` / ``SETP <n>,<v>``   control-loop setpoint
    ``RANGE?<n>`` / ``RANGE <n>,<r>`` heater output range (0 == heater off)
    ``HTR? <n>``                  heater output, percent
    ``HTRST? <n>``               heater status  -> ``0`` (no fault)

A light thermal model runs behind it: each input channel relaxes exponentially
toward its driving temperature (the loop setpoint while the heater range is
non-zero, otherwise ambient), with a little measurement noise.  The heater
outputs follow the control error.  It is deliberately crude -- enough to see
BUSY -> IDLE transitions, ramps and plausible heater readings, not a real
cryostat model.

Channel / loop wiring matches ``cfg/sim_CF-17_cfg.py``:

    input A  <- loop 1 (heater output 1)   T_HEx
    input B  <- loop 2 (heater output 2)   T_cold_probe    cold sample stick
    input C  <- loop 2                     T_hot_probe     hot sample stick
    input D  <- no loop                    T_cold_head     (free running)

CF-17 has a single sample-stick port: either the cold stick (input B) or the
hot stick (input C) is physically inserted, never both.  The ``stick`` parameter
selects which one; the other input then reports as a disconnected sensor
(``KRDG?`` -> 0, ``RDGST?`` -> 160), which the driver surfaces as an ERROR
status.  Inputs A and D are always connected.
"""

import math
import random
import time

from frappy.core import Communicator, Parameter, FloatRange, StringType
from frappy.errors import RangeError


# input channel -> control loop that drives it (None == free running)
CH_LOOP = {'A': 1, 'B': 2, 'C': 2, 'D': None}
# control loop -> the input channel its heater output mainly acts on
LOOP_MAIN_CH = {1: 'A', 2: 'B'}
# which input each selectable sample stick uses
STICK_CHANNEL = {'cold': 'B', 'hot': 'C'}
# RDGST? bitfield returned for a disconnected input: 128 (sensor units overrange)
# + 32 (temperature overrange) -- what an open sensor reads on a LS336
DISCONNECTED_RDGST = 160


class Ls336Sim(Communicator):
    """Simulated LakeShore 336 (protocol level)."""

    idn = Parameter('*IDN? response', datatype=StringType(),
                    default='LSCI,MODEL336,LSASIM336,2.7', readonly=False)
    stick = Parameter("which sample stick is inserted ('cold' -> input B, "
                      "'hot' -> input C); the other input reads as disconnected",
                      datatype=StringType(), default='cold', readonly=False)
    tau = Parameter('thermal time constant of the approach to setpoint',
                    datatype=FloatRange(0.1, unit='s'), default=25.0, readonly=False)
    ambient = Parameter('temperature approached when the heater is off',
                        datatype=FloatRange(0, unit='K'), default=295.0, readonly=False)
    coldhead = Parameter('temperature of the free-running cold-head channel D',
                         datatype=FloatRange(0, unit='K'), default=34.0, readonly=False)
    noise = Parameter('peak measurement noise added to every channel',
                      datatype=FloatRange(0, unit='K'), default=0.02, readonly=False)
    pollinterval = Parameter('advance interval', datatype=FloatRange(0.1),
                             default=1.0, readonly=False)

    _temp = None       # dict: channel letter -> current temperature / K
    _setp = None       # dict: loop number -> setpoint / K
    _range = None      # dict: loop number -> heater range (0..3, 0 == off)
    _htr = None        # dict: output number -> heater output / %
    _last = 0.0        # time.monotonic() of the last thermal advance

    def earlyInit(self):
        super().earlyInit()
        self._temp = {'A': 295.0, 'B': 295.0, 'C': 295.0, 'D': self.coldhead}
        self._setp = {1: 295.0, 2: 295.0}
        self._range = {1: 0, 2: 0}
        self._htr = {1: 0.0, 2: 0.0}
        self._last = time.monotonic()

    # ---- thermal model -------------------------------------------------------

    def _advance(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        if dt <= 0:
            return
        dt = min(dt, 5.0)  # never take a huge single step

        disconnected = self._disconnected()
        for ch, loop in CH_LOOP.items():
            if ch in disconnected:
                # no sensor: let the internal state drift gently to ambient so a
                # re-inserted stick starts from a sensible reading (the KRDG?/
                # RDGST? replies are overridden in _handle regardless)
                drive, tau = self.ambient, self.tau * 20
            elif loop is None:
                drive, tau = self.coldhead, self.tau * 3
            elif self._range[loop] > 0:
                drive, tau = self._setp[loop], self.tau
            else:
                drive, tau = self.ambient, self.tau * 6
            alpha = 1.0 - math.exp(-dt / max(tau, 0.1))
            t = self._temp[ch] + (drive - self._temp[ch]) * alpha
            t += random.uniform(-1.0, 1.0) * self.noise
            self._temp[ch] = max(0.0, t)

        for out in (1, 2):
            # the input this loop actually regulates (loop 2 follows the stick)
            ch = LOOP_MAIN_CH[out]
            if ch in disconnected:
                ch = STICK_CHANNEL[self.stick]
            if self._range[out] > 0:
                err = self._setp[out] - self._temp[ch]
                self._htr[out] = min(100.0, max(0.0, 3.0 * err + 5.0))
            else:
                self._htr[out] = 0.0

    def doPoll(self):
        super().doPoll()
        self._advance()

    def write_stick(self, value):
        value = str(value).strip().lower()
        if value not in STICK_CHANNEL:
            raise RangeError(f"stick must be one of {sorted(STICK_CHANNEL)}")
        return value

    def _disconnected(self):
        """set of input channels with no sensor connected right now"""
        return {ch for stick, ch in STICK_CHANNEL.items() if stick != self.stick}

    # ---- protocol ----------------------------------------------------------

    def communicate(self, command):
        self.comLog(f'> {command}')
        self._advance()
        replies = []
        for chunk in command.split(';'):
            chunk = chunk.strip()
            if not chunk:
                continue
            reply = self._handle(chunk)
            if reply is not None:
                replies.append(str(reply))
        reply = ';'.join(replies)
        self.comLog(f'< {reply}')
        return reply

    def _handle(self, chunk):
        if '?' in chunk:
            head, _, arg = chunk.partition('?')
            query, cmd, arg = True, head.strip().upper(), arg.strip()
        else:
            cmd, _, arg = chunk.partition(' ')
            query, cmd, arg = False, cmd.strip().upper(), arg.strip()

        if cmd == '*IDN':
            return self.idn
        if cmd == '*OPC':
            return '1'

        if cmd == 'KRDG':
            if arg.upper() in self._disconnected():
                return '+0.000'
            return f'{self._temp.get(arg.upper(), 0.0):+.3f}'
        if cmd == 'SRDG':  # sensor units - just hand back the kelvin value
            if arg.upper() in self._disconnected():
                return '+0.0000E+00'
            return f'{self._temp.get(arg.upper(), 0.0):+.4E}'
        if cmd == 'RDGST':
            if arg.upper() in self._disconnected():
                return str(DISCONNECTED_RDGST)
            return '0'

        if cmd == 'SETP':
            if query:
                return f'{self._setp.get(_intarg(arg), 0.0):+.3f}'
            loop, value = arg.split(',')
            self._setp[_intarg(loop)] = float(value)
            return None
        if cmd == 'RANGE':
            if query:
                return str(self._range.get(_intarg(arg), 0))
            loop, value = arg.split(',')
            self._range[_intarg(loop)] = int(float(value))
            return None

        if cmd == 'HTR':
            return f'{self._htr.get(_intarg(arg), 0.0):+.1f}'
        if cmd == 'HTRST':
            return '0'

        self.log.info('Ls336Sim: unhandled command %r', chunk)
        return '' if query else None


def _intarg(text):
    """first integer in *text* (tolerates stray spaces / trailing junk)."""
    return int(str(text).strip().split()[0])
