# Receive gain and analog bandwidth

`LIMITS?` extends protocol 6. `CAPS RXLIMITS` advertises it. Its JSON contains
`gain: [min,max,step]`, `bandwidth: [min,max,step,default]` or `null`, `rates`
in samples/second, and `bits` per I/Q component. The complete response fits
the existing 256-byte header limit. Gain units are table indices; bandwidth
units are MHz. `BANDWIDTH 0` means minimum capacitance, not filter bypass.

## Gain

C61 and S3's pinned PHY table writers populate AGCPWR_CTRL7 bits 8–14 with
the calibrated low-table maximum. C5's table writer calls its ROM
`phy_agc_max_gain_set` helper. The table capacities differ: C61 80 entries,
S3 83 entries, C5 90 entries. S31 already snapshots its generated table.
Do not replace these runtime limits with a single chip-independent constant.

C61 manual gain follows `sensor-firmware/main/iq/modem.c`: mirror the forced
low-table entry into slot index+80, set AGC initial gain/threshold and disable
RF saturation intervention. Without the mirror, DSSS classification can select
an uncalibrated second-table entry and change the gain unexpectedly. Restore
any overwritten calibrated high-table entry when leaving manual mode; entries
beyond that table's maximum are not used by hardware AGC. The capture wrapper
uses about 2 KiB of static storage, not a sample-sized allocation.

Hardware observations on 2026-09-28: C61 maximum 76, S3 maximum 82, S31
maximum 71. These values are observations, not a protocol guarantee.

## Analog filter curves

The firmware converts requested bandwidth to the nearest capacitor-DAC code
by piecewise linear interpolation. The browser must not duplicate these curves.
These are approximate two-sided noise-passband widths, not a calibrated RF
power measurement or a guaranteed anti-alias response at every sample rate.
Calibration varies between chips, boards and operating conditions.

| DAC code | C61 MHz | S31 MHz | S3 MHz |
| --- | --- | --- | --- |
| 0 | 54 | 54 | 69 |
| 4 | — | — | 51 |
| 8 | 36 | 36 | 45 |
| 16 | 30 | 30 | 33 |
| 24 | 21 | — | 25 |
| 28 | — | 21 | — |
| 32 | 18 | 18 | 21 |
| 48 | 15 | 15 | 16 |
| 60 | 13 | 13 | 13 |

C61's curve and BBTOP 4/5 register selection come from the known-good
`sensor-firmware/main/iq/modem.c`, with CBW20. S31 retains its existing
characterized 21 MHz anchor at code 28. C5 has no verified MHz curve and
advertises `bandwidth: null`; the viewer leaves filtering automatic.

The S3 curve was measured on the connected S3 at 2300 MHz, manual index 75,
80 MS/s: twelve 16,380-sample IQ8 captures per DAC code, seven 2048-point
Hann-windowed FFTs per capture, median power and 1 MHz spectral bins. The
reference level was the median noise power at ±2–5 MHz; widths use the
approximate −3 dB span. Code 63 gave no useful additional narrowing compared
with code 60, so the advertised range ends at 13 MHz. The firmware applies
registers 4/5 only around each capture and restores PHY calibration afterwards.

## Hardware checks

C61/S3/S31 were exercised over their USB-to-UART bridges at 2 MBaud: handshake,
hardware AGC, manual gain endpoints, rejection above the advertised maximum,
retuning, sample payload CRCs and bandwidth endpoints. The browser's actual
serial driver and UI were also driven through a serial test bridge to the boards.
S31 snapshots use native IQ8 from PARLIO at 16/8/4 MS/s and only join contiguous
source chunks. C5 has build/unit coverage; no connected C5 was available.

## C6 profile

C6 exposes IQ8/IQ10 at nominal 80 MS/s and an approximate 12–54 MHz analog-bandwidth range. Gain limits are read from the calibrated AGC register; the tested revision 0.1 reported indices 0–79.

RX analog filtering uses BBTOP I2C block 0x67, host 1, registers 4 and 5, low six bits. Each capture saves the calibrated register values, applies the requested code and restores the originals before sending data. `BANDWIDTH 0` selects code 0 (widest measured setting); `ALPF AUTO` restores automatic calibration behavior.

The C6 curve was measured separately from C61 at 2484 MHz, gain 79 and 80 MS/s: 24 snapshots per code, 2048-point Hann-window FFTs, median power across segments, both spectral sides combined. The approximate -3 dB full widths are rounded to whole MHz for control interpolation:

| Capacitor code | Approximate bandwidth (MHz) |
| --- | --- |
| 0 | 54 |
| 4 | 48 |
| 8 | 39 |
| 12 | 33 |
| 16 | 28 |
| 24 | 23 |
| 32 | 20 |
| 40 | 17 |
| 48 | 15 |
| 60 | 12 |

Code 63 measured approximately the same minimum width as code 60. These noise-passband measurements are not precision RF calibration and can vary between boards.

### C6 sample-rate investigation

Stock `adc_rate_set(0/1)` changed the expected registers (I2C ADC register 4: 0x6b/0x63; FE register 0x600a046c: 0x16100610/0x36100610), but source-15 capture timing and spectral scaling remained unchanged. The other two encodings of the same ADC clock field also retained that timing. 4096 samples took approximately 53–55 us; 16380 took 207–209 us, consistent with 80 MS/s plus fixed overhead.

All sixteen dump-source selectors were examined. Alternate sources either timed out or filled intermittently with variable timing, so none was accepted as a lower-rate, uniformly sampled IQ stream. Earlier candidate dump-divider bits likewise did not change the rate. This establishes the current supported capture path, not a proof that every possible C6 hardware configuration is limited to 80 MS/s. Unsupported rate indices remain rejected instead of mislabeling 80 MS/s data.


C6 extended tuning accepts whole MHz from 2100 through 2800, advertised as
`TUNEEXT` and `RANGE 2100 2800 1`. This is an attempt range, not a measured
RF operating range. Standard Wi-Fi centers use `chip_v7_set_chan`; other
frequencies first calibrate at 2412 MHz, then call `phy_set_freq(mhz, 0)`.
Calling the channel function directly for non-channel MHz would convert the
request through `mhz2ieee`, potentially changing the requested center.

The web app shows a persistent warning outside standard C6 Wi-Fi centers and
continues capture. It does not snap extended requests to channels. Older C6
firmware without `TUNEEXT` still requires an update before these requests can
be made. Malformed requests and values outside the advertised attempt range
remain errors.

Native USB hardware validation accepted 2412, 2413, 2402, 2426, 2480, 2390,
2500, 2300, 2600, 2200, 2700, 2100 and 2800 MHz, then returned to 2412 MHz.
Both IQ8 and IQ10 captures passed CRC at every setting (28 captures).
This checks command completion and data integrity, not absolute LO accuracy,
PLL lock or usable sensitivity at the range endpoints.

A receive-spectrum cross-check at 2412 and 2413 MHz found the strongest
correlation at -26 FFT bins (2048-point FFT at 80 MS/s; expected -25.6 bins).
This supports a real 1 MHz LO change rather than Wi-Fi channel rounding;
it is an ambient-signal check, not an absolute frequency calibration.
