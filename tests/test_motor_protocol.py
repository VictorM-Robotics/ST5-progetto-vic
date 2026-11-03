"""Python driver exchanges frames with the actual sketch using fake native I/O."""
import pathlib
import struct
import subprocess
import tempfile
import unittest
from motor_bridge import MotorBridge, MotorCommunicationError


class NativeSerial:
    def __init__(self, executable, **kwargs):
        self.process = subprocess.Popen([executable, '--serve'], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE)
        self.now = 0
        self.buffer = bytearray()
        self.mutate = None
        self.short_write = False

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self, frame):
        if self.short_write:
            return 0
        self.process.stdin.write(struct.pack('<I', self.now) + frame)
        self.process.stdin.flush()
        length = self.process.stdout.read(1)
        if not length:
            raise RuntimeError('Native firmware exited')
        reply = bytearray(self.process.stdout.read(length[0]))
        if self.mutate and reply:
            reply = self.mutate(reply)
        self.buffer.extend(reply)
        return len(frame)

    def read(self, length):
        # Exercise fragmented host reads too.
        length = min(length, 3)
        result = bytes(self.buffer[:length])
        del self.buffer[:length]
        return result

    def close(self):
        self.process.stdin.close()
        self.process.wait(timeout=3)
        self.process.stdout.close()


class MotorProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = pathlib.Path(__file__).resolve().parents[1]
        cls.tmp = tempfile.TemporaryDirectory()
        cls.executable = str(pathlib.Path(cls.tmp.name) / 'firmware')
        subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                        '-DST5_NATIVE_TEST', '-I'+str(root/'tests/firmware'),
                        str(root/'tests/firmware/test_watchdog.cpp'), '-o', cls.executable],
                       check=True, capture_output=True, text=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        self.serial = None
        def factory(**kwargs):
            self.serial = NativeSerial(self.executable, **kwargs)
            return self.serial
        self.bridge = MotorBridge(allow_hardware=True, serial_factory=factory, sleep=lambda _:None)
        self.bridge.connect()
        self.addCleanup(self.bridge.close)

    def test_connect_does_not_arm(self):
        with self.assertRaises(PermissionError):
            self.bridge.set_wheels(40,40,40,40)
        self.assertFalse(self.bridge._connected)

    def test_arm_drive_stop_and_telemetry(self):
        self.bridge.arm()
        self.bridge.set_wheels(40,40,40,40)
        self.assertEqual(self.bridge.get_encoder_counts(), (0,0,0,0))
        self.bridge.stop()
        self.assertTrue(self.bridge._armed)
        self.bridge.disarm()
        self.assertFalse(self.bridge._armed)
        with self.assertRaises(MotorCommunicationError): self.bridge.arm()

    def test_timeout_does_not_rearm(self):
        self.bridge.arm()
        self.bridge.set_wheels(40,40,40,40)
        self.serial.now = 350
        with self.assertRaisesRegex(MotorCommunicationError, 'status 1'):
            self.bridge.set_wheels(40,40,40,40)
        self.assertFalse(self.bridge._armed)
        self.assertFalse(self.bridge._connected)

    def test_telemetry_does_not_refresh_watchdog(self):
        self.bridge.arm()
        for now in (100,200,300,400):
            self.serial.now = now
            self.bridge.get_encoder_counts()
        with self.assertRaises(MotorCommunicationError):
            self.bridge.set_wheels(40,40,40,40)

    def test_corrupt_response_disarms_driver(self):
        self.bridge.arm()
        def corrupt(reply):
            reply[8] ^= 1
            return reply
        self.serial.mutate = corrupt
        with self.assertRaisesRegex(MotorCommunicationError, 'Corrupt'):
            self.bridge.get_encoder_counts()
        self.assertFalse(self.bridge._armed)

    def test_no_automatic_reconnect_on_partial_write(self):
        self.bridge.arm()
        self.serial.short_write = True
        with self.assertRaises(MotorCommunicationError):
            self.bridge.set_wheels(40,40,40,40)
        self.assertFalse(self.bridge._connected)
