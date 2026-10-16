"""The H2 firmware AGC converges on a simulated receiver with ~1 dB per gain step."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class H2Agc(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_converges_and_stays_in_range(self):
        source = (ROOT / 'main/targets/esp32h2/receiver.c').read_text()
        agc = source[source.index('#define AGC_MIN'):source.index('static bool acquire_iq(')]
        program = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
static uint32_t IQ_BUFFER[512];
static unsigned forced = 0, captures = 0;
static double input_db; /* signal level at gain index 0, in dB of full scale */
static void force_rx_gain(unsigned manual, unsigned index) { assert(manual == 1); forced = index; }
static void bt_rx_force(unsigned on) { assert(on == 1); }
/* Simulated receiver: 1 dB per index, clipping at the 10-bit full scale. */
static const char *dump_iq(unsigned n, unsigned divider, unsigned *us) {
    double amplitude = 511.0 * pow(10.0, (input_db + forced) / 20.0);
    int v = amplitude > 511 ? 511 : (int)amplitude;
    for (unsigned j = 0; j < n; j++) IQ_BUFFER[j] = (uint32_t)(j & 1 ? v : (1024 - v) & 1023) | ((uint32_t)v << 10);
    captures++; *us = 1; (void)divider;
    return NULL;
}
''' + agc + r'''
int main(void) {
    for (double level = -80; level <= 0; level += 5) {
        for (unsigned start = AGC_MIN; start <= AGC_MAX; start += 9) {
            input_db = level; agc_index = start; captures = 0;
            software_agc(6);
            assert(captures <= 6);
            assert(agc_index >= AGC_MIN && agc_index <= AGC_MAX);
            double peak = 511.0 * pow(10.0, (level + agc_index) / 20.0);
            /* Either in the target window, or pinned at a limit that cannot do better. */
            assert((peak >= 128 && peak < 448) || (agc_index == AGC_MAX && peak < 128) ||
                   (agc_index == AGC_MIN && peak >= 448));
        }
    }
    puts("ok");
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            c = Path(directory) / 'agc.c'
            c.write_text(program)
            binary = Path(directory) / 'agc'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wno-unused-function', str(c), '-lm', '-o', str(binary)],
                           check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
