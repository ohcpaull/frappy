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
"""Protocol-level simulator for a Pfeiffer TPG vacuum gauge controller.

Used *instead of* ``IO('TPG_io', 'tcp://...')``.  The real
``frappy_acns.pfeiffer.PfeifferTPGPressureSensor`` runs unchanged against it.

Implements only what that driver sends (the low-level ACK / ENQ handshake done
by ``PfeifferIO`` is transport framing and is simply not needed here):

    ``PR<n>``   pressure of gauge <n>   -> ``"<status>,<value>"``  (status 0 == ok)
    ``ERR``     error status            -> ``"0000"``             (no error)

The pressure does a slow random walk in log space around ``pressure``.
"""

import random

from frappy.core import Communicator, Parameter, FloatRange


class PfeifferTPGSim(Communicator):
    """Simulated Pfeiffer TPG pressure controller (protocol level)."""

    pressure = Parameter('centre pressure of the random walk',
                         datatype=FloatRange(0, unit='mbar'), default=5e-6,
                         readonly=False)
    walk = Parameter('random-walk step, in decades per reading',
                     datatype=FloatRange(0), default=0.03, readonly=False)

    _p = 0.0

    def earlyInit(self):
        super().earlyInit()
        self._p = self.pressure

    def communicate(self, command):
        self.comLog(f'> {command}')
        cmd = command.strip().upper()
        if cmd.startswith('PR'):
            # slow bounded random walk in log space around `pressure`
            self._p *= 10.0 ** random.uniform(-self.walk, self.walk)
            self._p += (self.pressure - self._p) * 0.05
            self._p = max(1e-11, self._p)
            reply = f'0,{self._p:.4E}'
        elif cmd == 'ERR':
            reply = '0000'
        else:
            self.log.info('PfeifferTPGSim: unhandled command %r', command)
            reply = ''
        self.comLog(f'< {reply}')
        return reply
