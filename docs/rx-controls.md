# Receive controls

Protocol 6 advertises `RXLIMITS` in `CAPS`. Query `LIMITS?` after `INFO` for:

- `gain`: `[minimum, maximum, step]` in PHY table indices.
- `bandwidth`: `[minimum, maximum, step, default]` in MHz, or `null` when no
  characterized MHz mapping is available.
- `rates`: supported nominal sample rates in samples/second.
- `bits`: supported precision per I/Q component.

Hardware AGC is the default; manual gain indices are not absolute gain in dB.

The Wi-Fi AGC is built to settle on packet preambles. It does not protect
the ADC from a strong *continuous* signal: with a B210 tone a few
centimetres from the antenna, every Wi-Fi chip clipped in AGC mode (peak at
full scale), and clipping products buried the tone. Use manual gain for strong
signals. On the H2, which has no usable AGC for this purpose, `GAIN HARDWARE`
runs a firmware AGC instead; see [esp32h2.md](esp32h2.md).

## Gain implementation

Gain maxima come from the calibrated PHY tables where available. C61 and S3
read AGCPWR_CTRL7 bits 8–14; C5 uses its PHY gain-table setup; S31 snapshots
its generated table. H2 reads the calibrated maximum from `phy_param[83]`
and keeps Bluetooth RX forced on when returning to hardware AGC.

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
| H2 | 4–11 MHz | 0 | 7 bits |
| C2 | 12–20 MHz | 4/5 | 6 bits |
| ESP32 | 12–67 MHz | 1/2 | 7 bits |
| C5 | 11–48 MHz | 6/7, two PHY modes | 6 bits |
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
reception at every sample rate. C5 uses PHY mode 0 for 11–23 MHz and mode 1
for 24–48 MHz or open. Changing modes runs the complete channel calibration,
including PBUS analog-control tables; retuning preserves the chosen mode. Its
curves include the digital-filter response.
ESP32's widest settings exceed the characterized span;
its numeric maximum uses code 8, while wide open selects code 0.

## Receive calibration

ESP-SDR uses the following calibration policy on all supported ESP32 families:

- **IQ imbalance:** Calibrate at startup and reuse those corrections across
  frequency changes. ESP-SDR does not repeat the loopback IQ sweep on retunes.
  H2 retains four startup corrections, one for each RF gain group.
- **DC offsets:** Remeasure at each new tuning frequency. If the PHY replaces
  DC calibration in the background, remeasure at the requested frequency
  before the next capture, even without another tuning command.
- **Repeated frequency requests:** Requesting the same frequency does not
  force another DC measurement unless the calibration has become stale.

This describes calibration controlled by ESP-SDR. The PHY can still perform
its own startup and background calibration, so “IQ calibration only once”
is not an absolute guarantee for everything inside the chip. Startup and
background calibration can still transmit; removing the retune IQ sweep does
not make the whole firmware RF-silent.

DC-measurement counters detect changes even when the PHY's temperature stamp
stays unchanged. ESP32 and S2 compare their cached DC tables instead because
internal PHY calls bypass the measurement wrappers. Recovery restores the
receive settings and uses the PHY mutex to serialize DC measurement with
tracking. The mutex is not held throughout acquisition: tracking can affect a
capture already in progress, with recovery at the next capture preparation.
S31 streaming stops acquisition, discards old DMA data, and starts a new stream
epoch on recovery. H2 and both S31 profiles currently disable periodic PLL
tracking; explicit PHY DC measurements still invalidate calibration.

## Rates and extended tuning

All burst targets advertise `TUNEEXT` and answer `RANGE?` with
`RANGE 100 6000 1`: any whole MHz from 100 to 6000 is accepted as a tuning
attempt (`main/common/rx_tuning.h`). Accepting a frequency does not mean the
PLL locked; the table below shows where reception actually works. C6 and C3
only support 80 MS/s; other rates are rejected.

C5 programs every frequency through `phy_set_chanfreq`. The other backends
calibrate on a standard Wi-Fi channel and then program the PLL directly, so
off-grid frequencies are not rounded to a channel.

### Measured reception range

A B210 transmitted a CW tone 2 MHz above the LO, over the air, with the
receiver in its default gain mode. "✓" means the tone was at least 15 dB
above the noise floor (typically 35–70 dB), "–" means it was not received.
Measured with [`tools/rf_tone_check.py`](../tools/rf_tone_check.py),
one board per chip, in October 2026 (`esp32s2` at 80 MS/s, others at
their lowest rate of at least 8 MS/s):

| Chip | 1850 | 1950 | 2100 | 2200 | 2300–2650 | 2750 | 5180–5800 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ESP32, C2, C3, C6, S2, S3 | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | |
| C5 | ✓ | – | – | ✓ | ✓ | – | ✓ |
| C61 | – | – | – | ✓ | ✓ | ✓ | |
| H2 | – | – | – | – | ✓ | – | |

On the S2, the 40 and 16 MS/s modes subsample the 80 MS/s stream without
filtering and lose roughly 15 dB of signal-to-noise ratio; use 80 MS/s where
possible.

### Lower-band LO conversion (5/6 divider)

Below 2210 MHz, ESP32, C2, C3, C6, S2 and S3 switch the LO to a 5/6 divider
(CKGEN `0x65:0[4]` on ESP32/S2/S3/C3, `0x62:16[3]` with host 1 on C2/C6).
`FREQ` still names the receive frequency: `FREQ 2001` programs a 2401.2 MHz PLL
coordinate. Calibration runs with the normal divider; the alternate divider is
applied after RX setup, and other frequencies restore normal conversion.
This follows the [eSpDR S3 investigation](https://github.com/h0m3us3r/eSpDR/commit/f279bf823eee41796dfd1ac21f13e1ed9b418c82);
external-tone measurements confirmed 5/6 (not the 8/9 suggested by older
vendor code).

Chip quirks: the ESP32 needs the SDK's patched `ram_chip_i2c_*` functions with
CKGEN host 4, must release its channel's fixed capacitor before a direct
retune, and takes its frequency offset in 1/1024 MHz rather than kHz. The S2
searches up to 512 capacitor codes and keeps the middle of the widest RFPLL
voltage window, restoring the previous fields if none is found.

C5, C61, H2 and S31 have different analog register layouts and no qualified
equivalent; `rx_lo.h` only accepts the six chips above. C61 and S31 instead
recover failed low-band PLL calibration automatically, which is why the C61
still receives at 2200 MHz.

The browser negotiates each chip's range and shows an informational warning
outside 2400–2483.5 MHz (and, on the C5, outside 5150–5895 MHz). It never
blocks tuning.

## Capture memory

ESP32 reserves a 64 KiB SRAM aperture at `0x3ffe8000`, with linker guards and
DPORT MAC_DUMP_MODE=3. Mode 2 only fills half the buffer. Packing occurs in place.

S2 reserves 48 KiB at `0x3fff0000–0x3fffc000` and its IRAM aliases, leaving the
top bank accessible to ROM USB. Its 12,284-sample maximum leaves four overrun
canaries. Source 0 supplies signed 10-bit I/Q; clock bits 15/16 select nominal
40/16 MS/s from the 80 MS/s source.

## GPIO outputs

Burst firmware advertises `GPIO` in `CAPS`. `GPIO?` returns a single line of
available output pins and their configured states, ordered by chip GPIO number:

```text
GPIO 0:Z 1:Z 2:Z 3:0 4:Z 14:1
```

The example is illustrative; clients must use the returned list rather than
infer pins from the chip name. An empty list is returned as `GPIO`.

`GPIO <pin> <Z|0|1>` changes one pin and replies `OK GPIO <pin> <state>`.
Invalid/unavailable pins, invalid states, and malformed requests return
`ERR gpio_args`; a driver failure returns `ERR gpio_io`.

- `Z`: high impedance, output and internal pull-up/pull-down disabled.
- `0`: push-pull output low.
- `1`: push-pull output high.

All advertised pins are initialized to Z once at firmware startup. Queries and
client connections do not change them. Settings survive retuning and client
disconnection, but reset to Z on reboot; they are not saved to flash. Responses
report the configured drive state, not the measured voltage at the pad.

Availability excludes input-only/nonexistent GPIOs, dedicated flash/PSRAM and
memory-supply pads (conservatively including optional memory), ESP-IDF-reserved
pins, native USB pins, and both UART pins when the UART transport is enabled.
Pin numbers are silicon GPIO numbers, not board connector labels. Firmware
cannot discover board wiring or which pads a particular module exposes.

GPIO commands obey the same serial ownership checks as receiver commands.
Clients must finish a burst, or stop and drain a spectrum stream, before sending
one. The browser provides a collapsed GPIO section at the bottom of the sidebar,
with a Z/0/1 button group for each available pin, and handles this sequencing.
