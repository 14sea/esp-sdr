#!/usr/bin/env python3
"""Check over-the-air reception with a USRP (e.g. B210) as test transmitter.

For each LO frequency, the USRP transmits a CW tone --offset Hz above it and
the board takes a CAP16 capture. The tone's level over the median noise floor
is measured within --tolerance of the expected offset, which keeps fixed board
spurs from being mistaken for it. Prints one JSON line per frequency.

Needs pyserial, numpy and UHD's Python module. Transmit only where you are
allowed to, at low power, into antennas next to the board.

Example:
    python tools/rf_tone_check.py --port /dev/ttyACM0 --freqs 2300,2442,2650
"""
import argparse
import json
import math
import threading
import time
import zlib

import numpy as np
import serial

RATES = [80000000, 40000000, 20000000, 10000000, 8000000, 4000000, 16000000, 32000000, 10666667, 6400000]


class Tone:
    """Continuous CW from a USRP, generated digitally at if_offset from its LO
    so the USRP's own LO leakage stays away from the test frequency."""

    def __init__(self, args, gain, if_offset=-1.5e6, rate=4e6):
        import uhd
        self.uhd, self.if_offset = uhd, if_offset
        self.usrp = uhd.usrp.MultiUSRP(args)
        self.usrp.set_tx_rate(rate)
        self.usrp.set_tx_gain(gain)
        n = 4000  # whole tone periods per buffer keep the phase continuous
        cycles = round(if_offset * n / rate)
        self.buffer = (0.3 * np.exp(2j * np.pi * cycles * np.arange(n) / n)).astype(np.complex64)
        self.streamer = self.usrp.get_tx_stream(uhd.usrp.StreamArgs('fc32', 'sc16'))
        self.running = True
        self.thread = threading.Thread(target=self._send, daemon=True)

    def tune(self, hz):
        self.usrp.set_tx_freq(self.uhd.types.TuneRequest(hz - self.if_offset))

    def _send(self):
        md = self.uhd.types.TXMetadata()
        while self.running:
            self.streamer.send(self.buffer, md)
        md.end_of_burst = True
        self.streamer.send(np.zeros(16, np.complex64), md)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.running = False
        self.thread.join()


class Board:
    def __init__(self, port):
        self.s = serial.Serial(port=None, baudrate=2000000, timeout=5)
        self.s.dtr = self.s.rts = False
        self.s.port = port
        self.s.open()
        time.sleep(2)  # some ports reset the board when opened
        marker = f'SYNC {time.monotonic_ns()}'
        self.s.reset_input_buffer()
        self.s.write(f'\nRELEASE\n{marker}\n'.encode())
        end = time.monotonic() + 5
        while self.s.readline().decode(errors='replace').strip() != marker:
            if time.monotonic() > end:
                raise TimeoutError('no reply from the board')

    def cmd(self, text):
        self.s.write((text + '\n').encode())
        return self.s.readline().decode().strip()

    def capture(self, n, rate):
        head = self.cmd(f'CAP16 {n} {RATES.index(rate)}').split()
        if head[0] != 'DATA':
            raise RuntimeError(' '.join(head))
        payload = self.s.read(int(head[1]) * 2)
        if zlib.crc32(payload) != int(head[2], 16):
            raise RuntimeError('CRC mismatch')
        v = np.frombuffer(payload, np.int8).astype(np.float32)
        return v[0::2] - 1j * v[1::2]  # Q is inverted so +f is above the LO


def tone_level(iq, fs, expect, tolerance):
    iq = iq - iq.mean()
    spec = np.abs(np.fft.fftshift(np.fft.fft(iq * np.hanning(len(iq))))) ** 2
    f = np.fft.fftshift(np.fft.fftfreq(len(iq), 1 / fs))
    near = np.abs(f - expect) <= tolerance
    k = int(np.flatnonzero(near)[np.argmax(spec[near])])
    return float(f[k]), 10 * math.log10(spec[k] / np.median(spec))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', required=True)
    ap.add_argument('--freqs', default='2412,2442,2472', help='LO frequencies in MHz')
    ap.add_argument('--offset', type=float, default=2e6, help='tone offset from the LO in Hz')
    ap.add_argument('--rate', type=int, help='sample rate (default: lowest advertised rate >= 4x offset)')
    ap.add_argument('--tolerance', type=float, default=50e3, help='allowed frequency error in Hz')
    ap.add_argument('--threshold', type=float, default=15, help='minimum tone level in dB')
    ap.add_argument('--usrp', default='type=b200', help='UHD device arguments')
    ap.add_argument('--tx-gain', type=float, default=50)
    a = ap.parse_args()

    board = Board(a.port)
    info = board.cmd('INFO').split()
    limits = json.loads(board.cmd('LIMITS?')[len('LIMITS '):])
    fs = a.rate or next((r for r in sorted(limits['rates']) if r >= 4 * a.offset), max(limits['rates']))
    samples = min(8192, int(info[3]))
    if 'HWAGC' in board.cmd('CAPS').split():
        board.cmd('GAIN HARDWARE')
    ok = True
    with Tone(a.usrp, a.tx_gain) as tone:
        for mhz in map(int, a.freqs.split(',')):
            tone.tune(mhz * 1e6 + a.offset)
            time.sleep(0.2)
            result = {'chip': info[0], 'lo_mhz': mhz, 'rate': fs, 'freq': board.cmd(f'FREQ {mhz}')}
            try:
                peak, level = tone_level(board.capture(samples, fs), fs, a.offset, a.tolerance)
                result.update(error_hz=round(peak - a.offset), level_db=round(level, 1), ok=level >= a.threshold)
            except Exception as e:  # report and continue with the next frequency
                result.update(error=str(e), ok=False)
            ok &= result['ok']
            print(json.dumps(result), flush=True)
    board.cmd('RELEASE')
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
