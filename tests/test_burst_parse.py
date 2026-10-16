"""Strict serial command number parsing rejects malformed and overflowing input."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

class BurstParse(unittest.TestCase):
    def test_strict_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / 'test-parse'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=undefined',
                            '-fno-sanitize-recover=undefined', '-I'+str(ROOT/'main/common'),
                            str(ROOT/'tests/burst_parse_test.c'), '-o', str(output)], check=True)
            subprocess.run([str(output)], check=True)

    def test_s3_commands_validate_before_receiver_changes(self):
        source = (ROOT/'main/targets/esp32s3/receiver.c').read_text()
        iqs = source[source.index('"IQS ",4)'):]
        self.assertLess(iqs.index('ERR args'), iqs.index('snprintf(h'))
        self.assertIn('burst_parse_int(line+5,-1000,1000', source)
        ring = (ROOT/'main/common/ring_capture.c').read_text()
        self.assertLess(ring.index('(d & (d - 1u))'), ring.index('while ((1u << l) < d)'))

if __name__ == '__main__':
    unittest.main()
