# On-chip spectra

`CAPS` advertises `SPEC SPECN SPECCAPS` only on enabled targets. Query
`SPECINFO?` before enabling a control. Its reply is `SPECINFO <JSON>`:

```json
{"continuous":false,"transports":["USB","UART"],"profiles":[[80000000,0,256,1,1,1],[80000000,0,512,1,1,0]]}
```

Each profile contains `[sample_rate_hz, rate_code, fft_bins, stride,
units_per_frame, continuous]`. The last field is optional for compatibility;
five-field profiles inherit the top-level `continuous` value. Capabilities
are specific to the connection: C3/C6/C61 use snapshot FFT on UART and
continuous 256-bin capture on native USB. S3 spectra require native USB.
Unsupported controls stay hidden. Existing S3 firmware without `SPECCAPS`
uses the original S3 compatibility profiles.

Send `SPEC <milliseconds> <stride> <units_per_frame> <detector> <rate_code>
<fft_bins>`. Zero milliseconds runs until a stop byte; detector 0 means mean
power and 1 means maximum power. Use the parameters from the chosen profile.
Portable snapshots require stride 1. C3/C6/C61 continuous frames contain one
FFT, so their two detector settings give the same individual-frame result;
the viewer still averages or maximizes successive frames for display.

The start reply is `SPEC <fft_bins> <sample_rate_hz> <unit_pairs> <MHz>`.
Binary frames follow. All integers are little-endian:

| Offset | Bytes | Meaning |
| --- | --- | --- |
| 0 | 4 | ASCII `SPC1` |
| 4 | 4 | Frame sequence |
| 8 | 8 | First sample index |
| 16 | 4 | Samples spanned by the frame |
| 20 | 2 | Completed FFTs represented |
| 22 | 1 | Flags: bit 0 maximum detector; bit 1 skipped processing; bit 2 prior output drop; bit 3 snapshot gaps |
| 23 | 1 | Gain metadata from the first I/Q word |
| 24 | 2 | Saturating cumulative frame-drop count |
| 26 | 1 | log2(FFT bins) |
| 27 | 1 | dB code multiplier (2) |
| 28 | N | Power codes in natural FFT order |
| 28+N | 4 | CRC32 of header and bins |

A power code represents `20*log10(power)` in the normalized Q15 FFT domain,
clamped to 0–255. The viewer converts this to dBFS using the Hann-window
normalization. Zero is a quantized floor, not evidence of absent RF energy.
Snapshot sample indices use elapsed wall time. C3 unwraps its hardware write
index against elapsed time across scheduler yields. Bank-rotation indices
come from the measured contiguous bank boundaries.

The end line has twelve decimal fields:

```
SPECEND status detail units pairs elapsed_us late_max work_max_cycles frames drops abandoned ffts stopped_by_host
```

Normal stops return status 0. A bank deadline/continuity failure stops capture
instead of silently presenting broken timing. Empty S3 frames are dropped.
A host that stops reading can lose a partial frame and the end record when
bounded output timeouts expire. The decoder resynchronizes on CRC-valid frames
or an end record; if it cannot establish the end boundary, it marks the
connection failed and requires reconnecting. It must not issue ordinary
commands into that uncertain stream. Reconnection reacquires the serial lease.

## Implementation constraints

S3 uses three SRAM banks and its SIMD FFT. C6 and C61 use two banks with scalar
FFT work split into short slices. These bank-rotation runs mask interrupts;
their firmware profiles disable interrupt/task watchdogs, as required by this
architecture. A stalled host ends the acquisition through the stream timeout.
C3 reads its live capture bank, masks interrupts only while copying one FFT
window, and yields during FFT processing. Its watchdog settings remain enabled.
All scalar FFT input copies are independent of subsequent RF writes.

ESP32, S2, C5 and S31 use snapshot FFTs because a safe continuous processing
path has not been established. Large S3 transforms at 40 MS/s can skip substantial
processing work; the stream counters report it. At 80 MS/s, the S3's bank-switch
deadline limits the current implementation to 256-bin FFTs.

## Validation

All advertised rate/FFT/detector combinations were checked on ESP32, C3, C5,
C6, C61, S2, S3 and S31, using UART on ESP32 and native USB on the others.
Checks covered frame CRCs, command access after capture, host stops, recovery
from reader stalls, and switching modes in the browser. Alternate UART wiring
on the native-USB boards was not tested. Continuous capture was checked using
hardware indices and bank boundaries, not calibrated RF phase coherence.

To repeat the profile check on an attached board, install `pyserial` and run:

```sh
python tools/check_spectrum.py --port /dev/ttyACM2 --milliseconds 3000
```

The tool does not flash firmware and emits a JSON report.
