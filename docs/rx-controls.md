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
`main/common/rx_bandwidth.h`. Zero selects the widest setting, not filter bypass.
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

All eight chips advertise `TUNEEXT` and answer `RANGE?` with
`RANGE 100 6000 1`: every integer MHz from 100 through 6000 is accepted for an
attempt. Fractional MHz and values outside that software range are rejected.
`main/common/rx_tuning.h` defines the shared limits. PLL lock is not a condition
for accepting a tuning command.

Out-of-channel requests calibrate on a standard channel before direct PLL
programming. C5 calibrates at 2412 MHz or 5180 MHz, selecting its 5 GHz path
above 3000 MHz, matching the pinned PHY's band selection. Its direct path uses
`phy_set_rf_freq_offset` with the calibrated crystal selector (`phy_param[49]`);
C5's `phy_set_freq` re-enters channel conversion and is deliberately bypassed.
S31 likewise keeps arbitrary frequencies out of channel calibration.

The browser negotiates ranges for every chip and uses them for text entry and
spectrum click-to-tune. Older firmware retains its advertised limits, with
legacy fallbacks only when it does not advertise `TUNEEXT`. An informational
warning appears outside 2400–2483.5 MHz; C5 also excludes its 5150–5895 MHz Wi-Fi band from the
warning. This never blocks tuning.

The 100–6000 MHz expansion is host-test/build verified only. The historical
hardware checks below covered the previous 2100–2800 MHz range; they do not
validate the new endpoints or RF performance.

ESP32 validation on an ESP32-D0WD-V3 rev. 3.1 with a 40 MHz crystal covered
all 701 whole-MHz settings with CRC-checked captures, plus 132 maximum-size
captures across 80/40/16 MS/s, 8/10-bit packing and manual gain/AGC. This
checks command and capture stability, not RF accuracy or PLL lock.

C61 validation on an ESP32-C61HR2 rev. 1.0 with a 40 MHz crystal likewise
covered all 701 settings with CRC-checked captures, plus 264 maximum-size
captures across all six rates, 8/10-bit packing and manual gain/AGC. RF
accuracy and PLL lock across the extended range remain unverified.

All advertised rates are nominal. Capture timing and payload checks do not
replace independent RF/sample-clock calibration.

## Capture memory

ESP32 reserves a 64 KiB SRAM aperture at `0x3ffe8000`, with linker guards and
DPORT MAC_DUMP_MODE=3. Mode 2 only fills half the buffer. Packing occurs in place.

S2 reserves 48 KiB at `0x3fff0000–0x3fffc000` and its IRAM aliases, leaving the
top bank accessible to ROM USB. Its 12,284-sample maximum leaves four overrun
canaries. Source 0 supplies signed 10-bit I/Q; clock bits 15/16 select nominal
40/16 MS/s from the 80 MS/s source.
