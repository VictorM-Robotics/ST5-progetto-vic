"""Conservative Raspberry Pi -> Arduino serial motor driver (ST5 VAC).

Bench-only pulse execution; never declares an intersection turn complete.
No physical actuation on import. Requires explicit hardware opt-in.
Requires firmware_watchdog/serial_link, protocol v2 (CRC). Not legacy A12.
"""
import struct
import binascii
import time
from dataclasses import dataclass


class MotorCommunicationError(RuntimeError):
    pass


@dataclass(frozen=True)
class MotorConfig:
    port: str = '/dev/ttyACM0'
    baudrate: int = 115200
    pwm_limit: int = 100  # deliberate conservative software cap, not a proven safe speed
    max_pulse_s: float = 0.3
    timeout_s: float = 0.4


class MotorBridge:
    def __init__(self, config=None, *, allow_hardware=False, serial_factory=None, sleep=time.sleep):
        self.config = config or MotorConfig()
        self.allow_hardware = allow_hardware
        self._factory = serial_factory
        self._sleep = sleep
        self._port = None
        self._connected = False
        self._armed = False
        self._sequence = 0

    def connect(self):
        if not self.allow_hardware:
            raise PermissionError('Hardware disabled: set allow_hardware=True only for a supervised bench test')
        if self._port is not None:
            raise RuntimeError('Already connected')
        if self._factory is None:
            import serial  # optional: only on Raspberry Pi / physical test
            self._factory = serial.Serial
        self._port = self._factory(port=self.config.port, baudrate=self.config.baudrate,
                                   timeout=self.config.timeout_s, write_timeout=self.config.timeout_s)
        try:
            self._sleep(2.0)  # Arduino resets on port open
            self._port.reset_input_buffer()
            self._sequence = 0
            hello = self._request('H')
            version, watchdog_ms, pwm_limit = struct.unpack('<BHH', hello[:5])
            if version != 2 or watchdog_ms != 350 or self.config.pwm_limit > pwm_limit:
                raise MotorCommunicationError('Incompatible motor firmware or PWM limit')
            self._connected = True
            self.stop()
            return True
        except Exception:
            self.close()
            raise

    def _frame(self, command, payload=b''):
        if len(payload) > 8:
            raise ValueError('Payload too long')
        self._sequence = (self._sequence + 1) & 0xffff
        body = struct.pack('<BBHB8s', 2, ord(command), self._sequence, 0, payload)
        return b'\xa5\x5a' + body + struct.pack('<H', binascii.crc_hqx(body, 0xffff))

    def _request(self, command, payload=b''):
        frame = self._frame(command, payload)
        if self._port.write(frame) != len(frame):
            raise MotorCommunicationError('Incomplete serial write')
        reply = bytearray()
        deadline = time.monotonic() + self.config.timeout_s
        while len(reply) < 17:
            if time.monotonic() >= deadline:
                raise MotorCommunicationError('Serial response timeout')
            part = self._port.read(17-len(reply))
            if not part:
                raise MotorCommunicationError('Incomplete serial response')
            reply.extend(part)
        if (reply[:2] != b'\xa5\x5a' or reply[2] != 2
                or struct.unpack('<H', reply[15:])[0] != binascii.crc_hqx(reply[2:15], 0xffff)
                or reply[3] != (ord(command) | 0x80)
                or struct.unpack('<H', reply[4:6])[0] != self._sequence):
            raise MotorCommunicationError('Corrupt, stale or incompatible motor response')
        if reply[6]:
            raise MotorCommunicationError(f'Motor firmware rejected command (status {reply[6]})')
        return bytes(reply[7:15])

    def arm(self):
        """Explicit operator/supervisor action. Never called automatically on timeout."""
        if not self._connected:
            raise MotorCommunicationError('Not connected')
        try:
            self._request('R')
            self._armed = True
        except Exception:
            self._best_effort_stop()
            raise

    def _send(self, pwms):
        if not self._connected:
            raise MotorCommunicationError('Not connected')
        if len(pwms) != 4 or any(type(x) is not int or abs(x) > self.config.pwm_limit for x in pwms):
            raise ValueError('Four signed integer PWM values within configured limit required')
        if any(pwms) and not self._armed:
            raise PermissionError('Motors disarmed: explicit arm() required')
        self._request('C', struct.pack('<4h', *pwms))

    def disarm(self):
        """Latched stop. Reconnect and explicitly arm before subsequent movement."""
        try:
            if self._connected:
                self._request('X')
        finally:
            self._armed = False
            self._connected = False

    def get_encoder_counts(self):
        """Read 4 cumulative signed 32-bit encoders in FR, FL, RR, RL order.

        Framed `N` returns front pair, `n` returns rear pair (not simultaneous).
        This call must not race another reader on the same serial port.
        """
        if not self._connected:
            raise MotorCommunicationError('Not connected')
        try:
            values = []
            for command in ('N', 'n'):
                raw = self._request(command)
                values.extend(struct.unpack('<2i', raw))
            return tuple(values)
        except Exception:
            self._best_effort_stop()
            raise

    def set_wheels(self, fr, fl, rr, rl):
        try:
            self._send((fr, fl, rr, rl))
        except Exception:
            self._best_effort_stop()
            raise

    def stop(self):
        self.set_wheels(0, 0, 0, 0)

    def _best_effort_stop(self):
        if self._port is not None:
            try:
                self._port.write(self._frame('X'))
            except Exception:
                pass  # if USB disconnects, only a firmware/hardware watchdog can stop motors
        self._armed = False
        self._connected = False

    def supervised_pulse(self, action, pwm=60, duration_s=0.15):
        """One bounded motor test pulse, NOT a 90-degree/180-degree manoeuvre.

        STOP is attempted in finally even on exception. Physical action requires
        wheels lifted, nearby operator, local emergency stop and verified direction.
        """
        if action not in ('LEFT', 'RIGHT', 'STRAIGHT', 'BACK', 'UTURN'):
            raise ValueError('Unsupported action')
        if type(pwm) is not int or not 0 < pwm <= self.config.pwm_limit:
            raise ValueError('PWM must be a positive integer below cap')
        if not isinstance(duration_s, (int, float)) or not 0 < duration_s <= self.config.max_pulse_s:
            raise ValueError('Duration exceeds bounded test interval')
        # Matches supplied arduino.py sign conventions; actual directions require verification.
        outputs = {
            'LEFT': (pwm, -pwm, pwm, -pwm),
            'RIGHT': (-pwm, pwm, -pwm, pwm),
            'STRAIGHT': (pwm, pwm, pwm, pwm),
            'BACK': (-pwm, -pwm, -pwm, -pwm),
            'UTURN': (pwm, -pwm, pwm, -pwm),
        }
        try:
            self.set_wheels(*outputs[action])
            self._sleep(duration_s)
        finally:
            self.stop()
        # No physical turn/line alignment confirmation is implied.
        return {'action': action, 'pulse_sent': True, 'maneuver_completed': False,
                'line_reacquired': False}

    def close(self):
        if self._port is not None:
            self._best_effort_stop()
            try:
                self._port.close()
            finally:
                self._port = None
                self._connected = False

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.close()
