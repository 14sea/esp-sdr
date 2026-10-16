"""The shared burst commands behave as documented and survive random input."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

PROGRAM = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define BURST_SERIAL_UART 1
#define BURST_ID "T1SDR"
#define BURST_MAX_RATE 6
#define IQ_WORDS 16380u
static int burst_serial_port(void) { return BURST_SERIAL_UART; }
static unsigned burst_serial_baud(void) { return 2000000; }
static unsigned frequency_mhz, captures, last_n, last_rate, last_format, prepared;
static bool rx_ready;
static char response[128];
static void reply(const char *s) { snprintf(response, sizeof(response), "%s", s); }
static void prepare_rx(void) { rx_ready = true; prepared++; }
static bool capture(unsigned n, unsigned rate, unsigned format) {
    captures++; last_n = n; last_rate = rate; last_format = format; reply("DATA\n"); return true;
}
static void vTaskDelay(int ticks) { (void)ticks; }
#include "rx_tuning.h"
static bool frequency_valid(unsigned mhz) { return mhz >= RX_FREQ_MIN && mhz <= RX_FREQ_MAX; }
#include "burst_commands.h"

static bool run(const char *text) {
    char line[160];
    snprintf(line, sizeof(line), "%s", text);
    response[0] = 0;
    return burst_common_command(line);
}

int main(void) {
    assert(run("INFO") && !strcmp(response, "T1SDR 6 burst 16380\n"));
    assert(run("TRANSPORT?") && !strcmp(response, "TRANSPORT UART 2000000\n"));
    assert(run("SYNC 18446744073709551615") && !strcmp(response, "SYNC 18446744073709551615\n"));
    assert(run("RANGE?") && !strcmp(response, "RANGE 100 6000 1\n"));
    assert(run("FREQ 2442") && frequency_mhz == 2442 && rx_ready && !strcmp(response, "OK\n"));
    assert(!run("FREQ 99") && !run("FREQ 6001") && !run("FREQ 2442 junk") && frequency_mhz == 2442);
    assert(run("CAP16 4096 0") && last_format == 16 && last_n == 4096);
    assert(run("CAP20 257 6") && last_format == 20 && last_rate == 6);
    assert(run("CAP 256 0") && last_format == 0);
    assert(!run("CAP16 255 0") && !run("CAP16 16381 0") && !run("CAP20 256 7") && !run("CAP20 256 0 junk"));
    unsigned before = captures;
    assert(run("RXRUN 256 0 3 16") && captures == before + 3 && !strcmp(response, "END\n"));
    assert(!run("RXRUN 256 0 0 16") && !run("RXRUN 256 0 1001 16") && !run("RXRUN 256 0 2 32"));
    assert(!run("CAPS") && !run("GAIN?") && !run(""));

    /* Random commands built from real words, numbers and junk. */
    static const char *words[] = {"INFO", "CAP", "CAP16", "CAP20", "RXRUN", "FREQ", "SYNC", "RANGE?", "TRANSPORT?", "x"};
    static const char *args[] = {"0", "6", "7", "255", "256", "16380", "16381", "4294967295", "4294967296",
                                 "-1", "99999999999999999999", "2442", "junk", " ", ""};
    unsigned seed = 1;
    for (unsigned i = 0; i < 100000; i++) {
        char line[160];
        int len = 0;
        seed = seed * 1103515245u + 12345u;
        len += snprintf(line, sizeof(line), "%s", words[(seed >> 8) % 10]);
        for (unsigned a = 0, count = (seed >> 20) % 6; a < count && len < 120; a++) {
            seed = seed * 1103515245u + 12345u;
            len += snprintf(line + len, sizeof(line) - len, " %s", args[(seed >> 8) % 15]);
        }
        run(line);
        assert(last_n >= 256 && last_n <= IQ_WORDS && last_rate <= BURST_MAX_RATE);
    }
    return 0;
}
'''


def sanitizer_flags():
    """Use ASan/UBSan where the host compiler and runtime support them."""
    flags = ['-fsanitize=address,undefined', '-fno-sanitize-recover=undefined']
    with tempfile.TemporaryDirectory() as directory:
        probe = Path(directory) / 'probe.c'
        probe.write_text('int main(void) { return 0; }\n')
        built = subprocess.run(['cc', *flags, str(probe), '-o', str(Path(directory) / 'probe')], capture_output=True)
        ran = built.returncode == 0 and subprocess.run([str(Path(directory) / 'probe')], capture_output=True).returncode == 0
    return flags if ran else []


class BurstCommands(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_commands_and_random_input(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'commands.c'
            source.write_text(PROGRAM)
            binary = Path(directory) / 'commands'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wno-unused-function', *sanitizer_flags(),
                            '-I' + str(ROOT / 'main/common'), str(source), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, env={'ASAN_OPTIONS': 'detect_leaks=0'})


if __name__ == '__main__':
    unittest.main()
