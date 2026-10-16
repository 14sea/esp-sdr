"""burst_capture.h packs IQ8/IQ10 in place exactly as the protocol documents."""
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def reference(words, fmt):
    """Independent packing: IQ8 keeps bits 2-9 and 12-19; IQ10 packs 20-bit words LSB first."""
    if fmt == 16:
        return bytes(b for w in words for b in ((w >> 2) & 255, (w >> 12) & 255))
    acc = sum((w & 0xFFFFF) << (20 * k) for k, w in enumerate(words))
    return acc.to_bytes((len(words) * 20 + 7) // 8, 'little')


class BurstCapture(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_packing_matches_reference(self):
        rng = random.Random(1)
        cases = [(n, [rng.getrandbits(32) for _ in range(n)]) for n in (1, 2, 3, 256, 257, 4096)]
        program = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
static uint32_t buffer[4096];
#define IQ_BUFFER buffer
static unsigned sent;
static bool send_bytes(const void *p, size_t n) { fwrite(p, 1, n, stdout); sent += n; return true; }
static uint32_t esp_rom_crc32_le(uint32_t crc, const uint8_t *p, size_t n) { (void)p; (void)n; return crc; }
#include "burst_capture.h"
int main(void) {
    unsigned n, fmt;
    while (scanf("%u %u", &n, &fmt) == 2) {
        for (unsigned j = 0; j < n; j++) scanf("%u", &buffer[j]);
        burst_send_capture(n, fmt, 0);
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            (tmp / 'esp_rom_crc.h').write_text('')
            (tmp / 'pack.c').write_text(program)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wno-unused-function', '-I' + str(tmp),
                            '-I' + str(ROOT / 'main/common'), str(tmp / 'pack.c'), '-o', str(tmp / 'pack')], check=True)
            for n, words in cases:
                for fmt in (16, 20):
                    stdin = f'{n} {fmt} ' + ' '.join(map(str, words))
                    out = subprocess.run([str(tmp / 'pack')], input=stdin.encode(), capture_output=True, check=True).stdout
                    header, payload = out.split(b'\n', 1)
                    self.assertEqual(header.split()[:2], [b'DATA', str(n).encode()])
                    self.assertEqual(payload, reference(words, fmt), f'n={n} format={fmt}')


if __name__ == '__main__':
    unittest.main()
