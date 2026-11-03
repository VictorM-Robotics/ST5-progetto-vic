import pathlib
import subprocess
import tempfile
import unittest


class FirmwareTests(unittest.TestCase):
    def test_actual_sketch_with_fake_arduino_io(self):
        root = pathlib.Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(pathlib.Path(tmp) / 'watchdog')
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined', '-DST5_NATIVE_TEST',
                            '-I'+str(root / 'tests/firmware'),
                            str(root / 'tests/firmware/test_watchdog.cpp'), '-o', binary],
                           check=True, capture_output=True, text=True)
            subprocess.run([binary], check=True, capture_output=True, text=True)
