"""Exercise the actual analog calibration against deterministic DAC plants."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

class DcCalibration(unittest.TestCase):
    def test_response_matrix_and_failure_rollback(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = pathlib.Path(directory)
            (tmp / 'freertos').mkdir()
            (tmp / 'cJSON.h').write_text('typedef struct cJSON cJSON;\n')
            (tmp / 'esp_err.h').write_text('typedef int esp_err_t;\n')
            (tmp / 'freertos/FreeRTOS.h').write_text('#define pdMS_TO_TICKS(x) (x)\n')
            (tmp / 'freertos/task.h').write_text('void vTaskDelay(unsigned ticks);\n')
            output = tmp / 'test-dc'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(tmp), '-I'+str(ROOT/'main/targets/esp32s31/streaming'),
                            str(ROOT/'main/targets/esp32s31/streaming/dc.c'),
                            str(ROOT/'tests/dc_calibration_test.c'), '-lm', '-o', str(output)], check=True)
            subprocess.run([str(output)], check=True)

if __name__ == '__main__':
    unittest.main()
