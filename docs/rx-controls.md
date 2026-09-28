# Receive controls

Protocol 6 advertises `RXLIMITS` in `CAPS`. Query `LIMITS?` after `INFO` for:

- `gain`: `[minimum, maximum, step]` in PHY table indices.
- `bandwidth`: `[minimum, maximum, step, default]` in MHz, or `null` when no
  characterized MHz mapping is available.
- `rates`: supported nominal sample rates in samples/second.
- `bits`: supported precision per I/Q component.

Hardware AGC is the default; manual gain indices are not absolute gain in dB.

## Gain implementation

Gain maxima come from the calibrated PHY tables where available. C61 and S3
read AGCPWR_CTRL7 bits 8–14; C5 uses its PHY gain-table setup; S31 snapshots
its generated table.

C61 manual gain follows `sensor-firmware/main/iq/modem.c`: mirror the forced
low-table entry into slot index+80, set AGC initial gain/threshold, and disable
RF saturation intervention. Restore overwritten entries and calibrated settings
when returning to hardware AGC. The mirror prevents DSSS classification from
selecting an uncalibrated second-table entry.

## Bandwidth implementation

`BANDWIDTH <MHz>` interpolates the chip's capacitor-code curve in
`main/rx_bandwidth.h`. Zero selects the widest setting, not filter bypass.
Out-of-range requests are rejected.

| Chip | Approximate bandwidth | BBTOP registers | Code width |
| --- | --- | --- | --- |
| ESP32 | 12–67 MHz | 1/2 | 7 bits |
| C5 | 11–23 MHz | 6/7 | 6 bits |
| C6 | 12–54 MHz | 4/5 | 6 bits |
| C61 / S31 | 13–54 MHz | 4/5 | 6 bits |
| S2 | 15–60 MHz | 4/5 | 6 bits |
| S3 | 13–69 MHz | 4/5 | 6 bits |

The register mappings use BBTOP block 0x67, I2C host 1. Capture code preserves
unrelated bits and restores calibrated registers before transfer or retuning,
including on capture errors. C61's curve derives from the reference sensor
firmware; other chips retain their own curves.

These are approximate receive-path noise widths. Board calibration, operating
conditions and digital filtering affect them; they do not guarantee alias-free
reception at every sample rate. In particular, C5's mapping includes the normal
digital-filter response. ESP32's widest settings exceed the characterized span;
its numeric maximum uses code 8, while wide open selects code 0.

## Rates and extended tuning

C6 currently exposes only nominal 80 MS/s. Other tested clock/divider settings
and dump sources did not establish a reliable lower-rate I/Q path. Unsupported
rates are rejected.

C6 and S2 advertise `TUNEEXT` and report whole-MHz attempt ranges through
`RANGE?`: 2100–2800 MHz and 2212–2813 MHz respectively. Standard Wi-Fi centers
use channel tuning. Other frequencies calibrate at 2412 MHz before programming
the PLL directly, avoiding channel-number rounding. The viewer warns and
continues capture outside standard centers. Requests outside the advertised
range remain errors. PLL lock, sensitivity and absolute frequency accuracy
are not guaranteed throughout these ranges.

All advertised rates are nominal. Capture timing and payload checks do not
replace independent RF/sample-clock calibration.

## Capture memory

ESP32 reserves a 64 KiB SRAM aperture at `0x3ffe8000`, with linker guards and
DPORT MAC_DUMP_MODE=3. Mode 2 only fills half the buffer. Packing occurs in place.

S2 reserves 48 KiB at `0x3fff0000–0x3fffc000` and its IRAM aliases, leaving the
top bank accessible to ROM USB. Its 12,284-sample maximum leaves four overrun
canaries. Source 0 supplies signed 10-bit I/Q; clock bits 15/16 select nominal
40/16 MS/s from the 80 MS/s source.

See [S31 capture details](s31-capture.md) for its memory and CPU handoff.
