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
"""A fake Caylar supply, speaking the real Ethernet protocol over TCP.

This is deliberately a *socket* simulator rather than a Frappy module: the point
is to exercise :mod:`frappy_acns.caylar` end to end, including ``CaylarIO`` and
its reply checking, which a module-level fake would bypass.  Run it and point a
node at it exactly as at the real supply::

    python -m frappy_acns.caylar_sim --port 1234 --config 80V140A

The physics is as simple as it can be while still producing the behaviour the
driver has to cope with: the output current slews towards the setpoint at the
ramp speed, and the field is the current times a fixed gauss-per-amp.  That is
enough to make ramping, arrival, limits and the power-off guard all behave, and
it is the behaviour a SICS or Grafana developer needs.  It is not a model of a
real magnet and does not pretend to be.
"""

import argparse
import socket
import socketserver
import threading

# gauss per amp - a made-up but plausible electromagnet, giving roughly
# +/-14 kG at the full 140 A of the 80V140A configuration
GAUSS_PER_AMP = 100.0

NATIVE_MAX_CURRENT = {'80V140A': 140.0, '90V30A': 30.0}
NATIVE_MAX_VOLTAGE = {'80V140A': 80.0, '90V30A': 90.0}


class FakeSupply:
    """The state of the simulated supply, and the physics that moves it.

    All access is under a lock: the socket handler and the ramp thread both
    touch this, and a half-updated reading is exactly the sort of thing that
    would send someone hunting for a protocol bug that is not there.
    """

    def __init__(self, power_config='80V140A'):
        self.lock = threading.RLock()
        self.power_config = power_config

        self.power = False
        self.fault = False
        self.fault_name = 'NO_ERROR'
        self.maintenance = False
        self.regul_mode = 'CURRENT'
        self.ramp_mode = 'DIGITAL'
        self.cmd_selec = 1  # 1 = internal (digital) setpoint

        self.current = 0.0
        self.current_setpoint = 0.0
        self.field_setpoint = 0.0

        # per power configuration, as on the real supply
        self.digital_current_ramp = {'80V140A': 5.0, '90V30A': 2.5}
        self.analog_current_ramp = {'80V140A': 10.0, '90V30A': 10.0}
        self.field_ramp = {'80V140A': 500.0, '90V30A': 250.0}
        self.user_max_current = dict(NATIVE_MAX_CURRENT)
        self.user_max_field = {'80V140A': 20000.0, '90V30A': 10000.0}

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def field(self):
        return self.current * GAUSS_PER_AMP

    @property
    def voltage(self):
        """a plausible output voltage: proportional to current, clamped"""
        vmax = NATIVE_MAX_VOLTAGE[self.power_config]
        imax = NATIVE_MAX_CURRENT[self.power_config]
        return max(-vmax, min(vmax, self.current / imax * vmax * 0.8))

    def target_current(self):
        """the current the supply is actually heading for, in either mode"""
        if self.regul_mode == 'FIELD':
            return self.field_setpoint / GAUSS_PER_AMP
        return self.current_setpoint

    def ramp_rate(self):
        """A/s the output may slew at

        Two limits apply and the slower wins.  The current ramp (analog or
        digital, depending on the ramp mode) is what the output stage can do.
        On top of that, in field regulation the field *setpoint* moves at its
        own G/s ramp, which converted through GAUSS_PER_AMP is a second limit
        on how fast the current follows.
        """
        if self.ramp_mode == 'DIGITAL':
            rate = self.digital_current_ramp[self.power_config]
        else:
            rate = self.analog_current_ramp[self.power_config]
        if self.regul_mode == 'FIELD':
            rate = min(rate, self.field_ramp[self.power_config] / GAUSS_PER_AMP)
        return rate

    def is_ramping(self):
        return abs(self.current - self.target_current()) > 1e-3

    def _run(self):
        """slew the current towards the setpoint, 10 times a second"""
        tick = 0.1
        while not self._stop.wait(tick):
            with self.lock:
                if not self.power:
                    # no output when the power is off
                    self.current = 0.0
                    continue
                step = self.ramp_rate() * tick
                error = self.target_current() - self.current
                if abs(error) <= step:
                    self.current = self.target_current()
                else:
                    self.current += step if error > 0 else -step

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)


class CaylarHandler(socketserver.StreamRequestHandler):
    """One connected client.  One request line in, one reply line out."""

    @property
    def supply(self):
        return self.server.supply

    def handle(self):
        for raw in self.rfile:
            line = raw.decode('ascii', 'replace').strip()
            if not line:
                continue
            try:
                reply = self.respond(line)
            except Exception as err:  # pylint: disable=broad-except
                reply = f'WRONGCOMMAND ({err})'
            self.wfile.write((reply + '\n').encode('ascii'))

    def respond(self, line):
        fields = line.split()
        command, args = fields[0], fields[1:]
        with self.supply.lock:
            if command == '*IDN?':
                # The manual (p.9) documents 'CAYLAR_<serial number>', but the
                # ANSTO unit answers with the bare serial number and no
                # 'CAYLAR_' prefix. Match the hardware, not the manual - the
                # driver's `identification` regex is written against this.
                return '8220_072_SIM'
            if command.startswith('GET_'):
                return self.do_get(command[4:])
            if command.startswith('SET_') or command == 'CLEAR_DEFAULT':
                return self.do_set(command, args)
        return 'WRONGCOMMAND'

    # ------------------------------------------------------------------ reads

    def do_get(self, item):
        """answer a GET as '<item>= <value> [<unit>]'"""
        sup = self.supply
        cfg = sup.power_config
        # per-configuration items: strip the suffix and look up that config's
        # stored value, exactly as the real supply keeps them separately
        for suffix in ('_80V140A', '_90V30A'):
            if item.endswith(suffix):
                return self.do_get_per_config(item[:-len(suffix)], item, suffix[1:])

        simple = {
            'CURRENT': f'{sup.current:+.6f} A',
            'FIELD': f'{sup.field:+.2f} G',
            'VOLTAGE': f'{sup.voltage:+.3f} V',
            'CURRENT_SETPOINT': f'{sup.current_setpoint:+010.5f} A',
            'FIELD_SETPOINT': f'{sup.field_setpoint:+08.1f} G',
            'POWER_STATE': f'{int(sup.power)}',
            'POWER_CONFIGURATION': f'{0 if cfg == "80V140A" else 1}',
            'DEFAULT_STATE': f'{int(sup.fault)}',
            'DEFAULT_NAME': sup.fault_name,
            'MAINTENANCE_STATE': f'{int(sup.maintenance)}',
            'CMD_SELEC': f'{sup.cmd_selec}',
            'REGUL_MODE': sup.regul_mode,
            'RAMP_MODE': sup.ramp_mode,
            'FIELD_REGUL_STATE': '1',
            'DIGITAL_RAMP_STATE': f'{int(sup.ramp_mode == "DIGITAL" and sup.is_ramping())}',
            'ACTUAL_CURRENT_RAMP_SPEED': f'{sup.ramp_rate():05.2f} A/Sec',
            'BOX_TEMP': '+31.20 Deg',
            'RACK_TEMP': '+29.80 Deg',
            'WATER_TEMP': '+18.05 Deg',
            'HALL_PROBE_TEMP': '+45.468 Deg',
            'CURRENT_ADC_TEMP': '+45.123 Deg',
            'FIELD_ADC_TEMP': '+45.123 Deg',
            'WATER_FLOW': '+8.5 L/Min',
            'CURRENT_SETPOINT_INTERVAL':
                f'{-NATIVE_MAX_CURRENT[cfg]:.3f} {NATIVE_MAX_CURRENT[cfg]:.3f} A',
            'FIELD_SETPOINT_INTERVAL':
                f'{-sup.user_max_field[cfg]:.1f} {sup.user_max_field[cfg]:.1f} G',
        }
        if item in simple:
            return f'{item}= {simple[item]}'
        return 'WRONGCOMMAND'

    # How the reply to a per-configuration GET spells the configuration suffix.
    # The supply is inconsistent about this, command by command, and these are
    # the three behaviours actually observed:
    #
    #   'keep'    echo the suffix exactly as it was sent
    #   'drop'    leave the suffix off the echo entirely (documented in the
    #             manual for the two current ramp speeds)
    #   'respell' echo it with an underscore the command did not have, e.g.
    #             GET_FIELD_RAMP_SPEED_90V30A -> FIELD_RAMP_SPEED_90V_30A=
    #
    # Reproduced here on purpose: a tidy simulator that always echoed the
    # suffix back unchanged would never exercise the driver's handling of the
    # other two, which is where the real failures came from.
    GET_SUFFIX_STYLE = {
        'DIGITAL_CURRENT_RAMP_SPEED': 'drop',
        'ANALOG_CURRENT_RAMP_SPEED': 'drop',
        'FIELD_RAMP_SPEED': 'respell',
        'USER_MAX_CURRENT': 'keep',
        'USER_MAX_FIELD': 'keep',
    }

    @staticmethod
    def respell(cfg):
        """'90V30A' -> '90V_30A', the spelling the supply uses in some echoes"""
        return cfg.replace('V', 'V_', 1)

    def do_get_per_config(self, base, item, cfg):
        sup = self.supply
        values = {
            'DIGITAL_CURRENT_RAMP_SPEED': f'{sup.digital_current_ramp[cfg]:05.2f} A/Sec',
            'ANALOG_CURRENT_RAMP_SPEED': f'{sup.analog_current_ramp[cfg]:05.2f} A/Sec',
            'FIELD_RAMP_SPEED': f'{sup.field_ramp[cfg]:07.2f} G/Sec',
            'USER_MAX_CURRENT': f'{sup.user_max_current[cfg]:08.4f} A',
            'USER_MAX_FIELD': f'{sup.user_max_field[cfg]:07.1f} G',
        }
        if base not in values:
            return 'WRONGCOMMAND'
        style = self.GET_SUFFIX_STYLE[base]
        if style == 'drop':
            echo = base
        elif style == 'respell':
            echo = f'{base}_{self.respell(cfg)}'
        else:
            echo = item
        return f'{echo}= {values[base]}'

    # ----------------------------------------------------------------- writes

    def do_set(self, command, args):  # pylint: disable=too-many-return-statements
        """answer a SET as '<command>_OK [<readback>]' or '<command>_ERROR <reason>'

        A flat chain of cases, one per command.  Splitting it up would hide
        which reply belongs to which command, and that mapping is the whole
        point of this class.
        """
        sup = self.supply
        cfg = sup.power_config

        # maintenance mode blocks every SET, which is the behaviour a driver is
        # most likely to get wrong, so the simulator enforces it first
        if sup.maintenance and command != 'SET_MAINTENANCE_ON':
            return f'{command}_ERROR MAINTENANCE_ON'

        if command == 'CLEAR_DEFAULT':
            sup.fault = False
            sup.fault_name = 'NO_ERROR'
            return 'CLEAR_DEFAULT_OK'
        if command == 'SET_POWER_ON':
            if sup.fault:
                return 'SET_POWER_ON_ERROR DEFAULT_ON'
            sup.power = True
            return 'SET_POWER_ON_OK'
        if command == 'SET_POWER_OFF':
            sup.power = False
            return 'SET_POWER_OFF_OK'
        if command == 'SET_MAINTENANCE_ON':
            sup.maintenance = True
            return 'SET_MAINTENANCE_ON_OK'
        if command == 'SET_DEFAULT_ON':
            # a user fault cuts the power, like any other fault
            sup.fault = True
            sup.fault_name = 'USER_DEFAULT'
            sup.power = False
            return 'SET_DEFAULT_ON_OK'
        if command == 'SET_DEFAULT_OFF':
            # releases the condition, but the fault stays latched until
            # CLEAR_DEFAULT - the behaviour the manual is explicit about
            sup.fault_name = 'USER_DEFAULT'
            return 'SET_DEFAULT_OFF_OK'
        if command == 'SET_REGUL_CURRENT':
            sup.regul_mode = 'CURRENT'
            return 'SET_REGUL_CURRENT_OK'
        if command == 'SET_REGUL_FIELD':
            if sup.cmd_selec == 0:
                return 'SET_REGUL_FIELD_ERROR EXTERNAL_SETPOINT_SELEC'
            sup.regul_mode = 'FIELD'
            return 'SET_REGUL_FIELD_OK'
        if command in ('SET_FIELD_REGUL_RUN', 'SET_FIELD_REGUL_PAUSE'):
            if sup.regul_mode != 'FIELD':
                return f'{command}_ERROR CURRENT_REGUL'
            return f'{command}_OK'
        if command == 'SET_RAMP_MODE':
            if not args or args[0] not in ('ANALOG', 'DIGITAL'):
                return 'SET_RAMP_TYPE_ERROR BAD_ARG'
            sup.ramp_mode = args[0]
            return f'SET_RAMP_MODE_OK {sup.ramp_mode}'

        if command == 'SET_CURRENT':
            if sup.regul_mode == 'FIELD':
                return 'SET_CURRENT_ERROR FIELD_REG'
            value = self.as_float(args)
            if value is None:
                return 'SET_CURRENT_ERROR BAD_ARG'
            if abs(value) > sup.user_max_current[cfg]:
                return 'SET_CURRENT_ERROR OVERRANGE'
            sup.current_setpoint = value
            return f'SET_CURRENT_OK {value:+.6f} A'
        if command == 'SET_FIELD':
            if sup.regul_mode == 'CURRENT':
                return 'SET_FIELD_ERROR CURRENT_REG'
            value = self.as_float(args)
            if value is None:
                return 'SET_FIELD_ERROR BAD_ARG'
            if abs(value) > sup.user_max_field[cfg]:
                return 'SET_FIELD_ERROR OVERRANGE'
            sup.field_setpoint = value
            return f'SET_FIELD_OK {value:+.1f} G'

        return self.do_set_per_config(command, args)

    def do_set_per_config(self, command, args):
        """the '..._80V140A <value>' family of settings"""
        sup = self.supply
        for suffix in ('_80V140A', '_90V30A'):
            if not command.endswith(suffix):
                continue
            base, cfg = command[:-len(suffix)], suffix[1:]
            value = self.as_float(args)
            if value is None:
                return f'{command}_ERROR BAD_ARG'
            stores = {
                'SET_DIGITAL_CURRENT_RAMP_SPEED': (sup.digital_current_ramp, 99.9, '04.1f', 'A/Sec'),
                'SET_FIELD_RAMP_SPEED': (sup.field_ramp, 9999.99, '07.2f', 'G/Sec'),
                'SET_USER_MAX_CURRENT': (sup.user_max_current, NATIVE_MAX_CURRENT[cfg], '08.4f', 'A'),
                'SET_USER_MAX_FIELD': (sup.user_max_field, 99999.0, '07.1f', 'G'),
            }
            if base not in stores:
                return 'WRONGCOMMAND'
            store, limit, fmt, unit = stores[base]
            if value < 0 or value > limit:
                return f'{command}_ERROR OVERRANGE'
            store[cfg] = value
            return f'{command}_OK {value:{fmt}} {unit}'
        return 'WRONGCOMMAND'

    @staticmethod
    def as_float(args):
        """the supply's single numeric argument, or None if it is not one"""
        try:
            return float(args[0])
        except (IndexError, ValueError):
            return None


class CaylarServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True
    address_family = socket.AF_INET

    def __init__(self, address, power_config='80V140A'):
        self.supply = FakeSupply(power_config)
        super().__init__(address, CaylarHandler)

    def server_close(self):
        super().server_close()
        self.supply.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1234)
    parser.add_argument('--config', default='80V140A', choices=sorted(NATIVE_MAX_CURRENT))
    parser.add_argument('--power-on', action='store_true',
                        help='start with the output already on')
    args = parser.parse_args()

    server = CaylarServer((args.host, args.port), args.config)
    server.supply.power = args.power_on
    print(f'fake Caylar {args.config} supply listening on {args.host}:{args.port}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
