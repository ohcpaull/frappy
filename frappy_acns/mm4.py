"""Frappy (SECoP) driver for the mm4 pressure controller.

The mm4 is reached over its text port (TCP 8888). Three modules share one
connection (names as in cfg/mm4_cfg.py):

    pressure       value = pressure (mbar), target = setpoint (starts AUTO),
                   control_active = AUTO, stop, reset (faults), _ok_warning
    valve          value = valve % open, target = valve % (ends AUTO),
                   controlled_by = self or pressure
    room_pressure  value = room pressure (mbar), if the barometer is fitted

The SECoP side follows the standard controller pattern: the pressure module
controls the valve module (control_active, controlled_by), and writing
either target takes control, as SECoP expects.

It uses the remote command set, which the firmware gets in the same release
as this driver. Every command gets one reply line: a query its words and
value (PRES? -> PRES 4.625 mbar), a set command nothing unless refused
(ERR <code> <text>), so each set line here ends with a query.
Each reading is its own query
(PRES?, PRES SP?, MODE?, VALV?, VALV MOVING?, BARO?, ALRM?), and each poll
chains its queries on one line with ';'. Nothing here
homes the valve, calibrates, changes a setting or drives the motor. Those
stay on the touch screen.
"""

from frappy.core import BUSY, ERROR, IDLE, WARN, BoolType, Command, Drivable, \
    FloatRange, HasIO, Parameter, Readable, StringIO, StringType, nopoll
from frappy.mixins import HasControlledBy, HasOutputModule
from frappy.errors import CommunicationFailedError, ImpossibleError


class Mm4IO(StringIO):
    """Line link to the mm4 text port: one reply line per command."""
    end_of_line = '\n'   # the board ends lines with CR LF; CR is stripped
    timeout = 3
    identification = [('*IDN?', r'MercShore,iPC,')]

    def do(self, command):
        """Send one line and read its reply; an ERR is raised to the client.
        Set commands reply only when refused, so a line that sets something
        ends with a query (MODE AUTO;MODE?) to always get one line back."""
        reply = self.communicate(command).strip()   # the CR of CR LF
        if reply.startswith('ERR'):
            raise ImpossibleError(reply[3:].strip())
        return reply

    def get(self, *queries):
        """Values of one-value queries, sent as one chained line, without
        their leading words or unit: get('PRES?', 'MODE?') -> ['4.625',
        'AUTO']. None where the board answers NONE."""
        parts = self.do(';'.join(queries)).split(';')
        for part in parts:
            if part.startswith('ERR'):
                raise ImpossibleError(part[3:].strip())
        if len(parts) != len(queries):
            raise CommunicationFailedError(f'{len(parts)} replies to {len(queries)} queries')
        values = []
        for query, part in zip(queries, parts):
            words = query[:-1]
            if not part.startswith(words + ' '):
                raise CommunicationFailedError(f'unexpected reply to {query}: {part}')
            value = part[len(words) + 1:]
            if value == 'NONE':
                value = None
            elif value.endswith((' mbar', ' %')):
                value = value.rsplit(' ', 1)[0]
            values.append(value)
        return values


def number(text):
    """A reading as a float, None when the board had no value."""
    return None if text is None else float(text)


def alarm_tags(text):
    """Tags of the active alarms from ALRM?, faults first ('' if none)."""
    return '' if text is None else ','.join(a.split(',')[0] for a in text.split('|'))


class Pressure(HasOutputModule, HasIO, Drivable):
    """Pot pressure and its control (SECoP: a controller with an output module).

    Writing target sets the setpoint and starts AUTO (control_active true).
    stop ends AUTO and holds the valve. Writing the valve's target ends AUTO
    first (the valve's controlled_by goes back to self).
    """

    ioClass = Mm4IO

    value = Parameter('pressure', FloatRange(unit='mbar', fmtstr='%.3f'))
    target = Parameter('setpoint; writing it starts AUTO',
                       FloatRange(0, 1000, unit='mbar', fmtstr='%.1f'), readonly=False)
    control_active = Parameter('true in AUTO (the board regulates the pressure)', BoolType(),
                               default=False)
    alarm = Parameter('active faults (F-n) and warnings (W-n), empty if none', StringType())
    tolerance = Parameter('within this of the setpoint the status is IDLE in AUTO',
                          FloatRange(0, unit='mbar'), readonly=False, default=0.5)

    def read_value(self):
        p, sp, mode, alarm = self.io.get('PRESSURE?', 'SETPOINT?', 'MODE?', 'ALARM?')
        p, sp = number(p), number(sp)
        if sp is not None:
            self.target = sp
        self.alarm = alarm_tags(alarm)
        self.sync_control(mode == 'AUTO')
        self.status = self.status_of(p, sp, mode, self.alarm)
        return p if p is not None else self.value

    def sync_control(self, auto):
        """Follow the board's mode, without sending anything."""
        self.control_active = auto
        out = self.output_module
        if out:
            if auto and out.controlled_by != self.name:
                out.controlled_by = self.name
            elif not auto and out.controlled_by:
                out.controlled_by = 0   # self

    def status_of(self, p, sp, mode, alarm):
        if p is None:
            return ERROR, 'no pressure reading'
        if alarm.startswith('F'):
            return ERROR, f'fault {alarm} (see the screen; reset once cleared)'
        if mode == 'HOMING':
            return BUSY, 'valve homing'
        if mode == 'NOT HOMED':
            return WARN, 'valve not homed: home it at the screen'
        if alarm:
            return WARN, f'warning {alarm}'
        if mode == 'AUTO':
            if sp is not None and abs(p - sp) > self.tolerance:
                return BUSY, 'AUTO, going to the setpoint'
            return IDLE, 'AUTO'
        return IDLE, 'MANUAL'

    def write_target(self, value):
        self.io.do(f'SETPOINT {value:.1f};MODE AUTO;MODE?')
        self.activate_control()
        return value

    def set_control_active(self, active):
        if self.control_active and not active:
            self.io.do('MODE MANUAL;MODE?')
        self.control_active = active

    @nopoll
    def read_control_active(self):
        self.read_value()
        return self.control_active

    @nopoll
    def read_alarm(self):
        self.read_value()
        return self.alarm

    @Command()
    def stop(self):
        """Stop the valve now; in AUTO this ends AUTO (MANUAL)."""
        self.io.do('STOP;MODE?')
        self.sync_control(False)

    @Command()
    def reset(self):
        """Clear latched faults whose cause has gone."""
        self.io.do('ALARM RESET;ALARM?')

    @Command()
    def ok_warning(self):
        """Acknowledge the warning, as OK on the screen."""
        self.io.do('ALARM OK;ALARM?')


class Valve(HasControlledBy, HasIO, Drivable):
    """Needle valve position. Writing target ends AUTO first (MANUAL)."""

    ioClass = Mm4IO

    value = Parameter('valve position', FloatRange(unit='%', fmtstr='%.2f'))
    target = Parameter('valve position to go to; writing it ends AUTO',
                       FloatRange(0, 100, unit='%', fmtstr='%.1f'), readonly=False)

    def read_value(self):
        v, mode, moving = self.io.get('VALVE?', 'MODE?', 'VALVE MOVING?')
        v = number(v)
        if v is None:
            self.status = WARN, 'valve not homed: home it at the screen'
            return self.value
        if mode == 'HOMING':
            self.status = BUSY, 'valve homing'
        elif moving == '1':
            self.status = BUSY, 'moving'
        else:
            self.status = IDLE, mode
        return v

    def write_target(self, value):
        self.self_controlled()   # ends AUTO through the pressure module
        self.io.do(f'VALVE SET {value:.1f};VALVE MOVING?')
        self.status = BUSY, 'moving'
        return value

    @Command()
    def stop(self):
        """Stop the valve now; in AUTO this ends AUTO (MANUAL)."""
        self.io.do('STOP;MODE?')


class RoomPressure(HasIO, Readable):
    """Room pressure from the barometer, if fitted."""

    ioClass = Mm4IO

    value = Parameter('room pressure', FloatRange(unit='mbar', fmtstr='%.2f'))

    def read_value(self):
        baro = number(*self.io.get('BAROMETER?'))
        if baro is None:
            self.status = ERROR, 'no reading (barometer not fitted?)'
            return self.value
        self.status = IDLE, ''
        return baro