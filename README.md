# ESP-SDR firmware

Receive-only I/Q snapshot firmware for ESP32 chips, with a browser viewer and
installer in [esp-web-sdr](../esp-web-sdr/README.md). The firmware uses internal
RAM and needs no PSRAM, display, status LED, buttons, or board-revision setup.

**Parts of the firmware code are AI-generated.**

## Chip support

| Chip | Status | Native USB | UART0 TX / RX | Minimum flash |
| --- | --- | --- | --- | --- |
| ESP32 | Supported | — | GPIO1 / GPIO3 | 2 MB |
| ESP32-C2 | 🚧 Unsupported | — | — | — |
| ESP32-C3 | 🚧 Unsupported | — | — | — |
| ESP32-C5 | Supported | Serial/JTAG | — | 2 MB |
| ESP32-C6 | Supported | Serial/JTAG | GPIO16 / GPIO17 | 2 MB |
| ESP32-C61 | Supported | Serial/JTAG | GPIO11 / GPIO10 | 2 MB |
| ESP32-H2 | 🚧 Unsupported | — | — | — |
| ESP32-H21 | 🚧 Unsupported | — | — | — |
| ESP32-H4 | 🚧 Unsupported | — | — | — |
| ESP32-P4 | 🚧 Unsupported; no integrated radio | — | — | — |
| ESP32-S2 | Supported | USB-OTG CDC | GPIO43 / GPIO44 | 4 MB |
| ESP32-S3 | Supported | Serial/JTAG | GPIO43 / GPIO44 | 2 MB |
| ESP32-S31 | Supported | Serial/JTAG | GPIO58 / GPIO59 | 2 MB |

Families follow [Espressif's chip overview](https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/get-started/board-selection.html).
Transport and flash columns describe this firmware, not every chip's hardware.
🚧 means no ESP-SDR backend is available; it does not promise a future port.

Each chip needs its own RF backend; selecting another ESP-IDF target does not
add support. Images work on larger flash devices without expanding partitions.
Captures have gaps: ADC sample rate is not sustained USB/UART throughput.
Gain and power measurements are uncalibrated.

## Connect

Use native USB or a 3.3 V USB-to-UART adapter with crossed TX/RX and common
ground. UART defaults to **2,000,000 baud, 8N1, no flow control** on the pins
above. Native USB ignores the host baud setting.

The `ESP_SDR_UART_ENABLED`, `ESP_SDR_UART_BAUD`, `ESP_SDR_UART_TX_PIN` and
`ESP_SDR_UART_RX_PIN` menuconfig options control UART. Disabling optional UART
leaves its pins unused by the application and keeps native USB available.
S2 UART support is build-verified but untested with an adapter; older USB-only
S2 images must be reflashed before using UART.

Both ports may be connected, but one client owns the radio. Other clients
receive `ERR busy` until `RELEASE` or five seconds of idle time.
`CAPS SERIALLEASE` advertises this behavior; `DUALSERIAL` indicates UART
availability alongside USB. `TRANSPORT?` returns `TRANSPORT USB 0` or
`TRANSPORT UART <baud>`.

## Build and flash

[firmware-targets.json](firmware-targets.json) lists the supported profiles and
pins their ESP-IDF commits, including the preview SDK for S31. Check out the
matching SDK, initialize its submodules, run `install.sh <target>`, and source
`export.sh`. These backends do not need this repository's PHY submodules.

Use a separate build directory and configuration for each chip:

```sh
idf.py -B build-s3 -DIDF_TARGET=esp32s3 \
  -DSDKCONFIG=sdkconfig.s3 \
  -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32s3 build
idf.py -B build-s3 -p /dev/ttyACM0 flash
```

Substitute the target and paths for your chip. S31 requires the pinned preview
SDK and `idf.py --preview`. Keep large build directories on disk rather than
in a RAM-backed `/tmp`.

## Receive controls and chip limits

All backends start with hardware AGC. `GAIN HARDWARE` restores it;
`GAIN MANUAL <index>` selects a PHY gain-table entry. Protocol 6 advertises
`RXLIMITS` in `CAPS`; query `LIMITS?` after `INFO` for supported gain indices,
bandwidths, sample rates and bit depths. The web viewer uses these limits.

`BANDWIDTH <MHz>` sets approximate analog bandwidth; zero selects the widest
setting. Out-of-range values are rejected. Bandwidth mappings and gain behavior
are documented in [receive-control notes](docs/rx-controls.md).

- **ESP32:** 2412–2472 MHz in 5 MHz steps; nominal 80/40/16 MS/s snapshots.
- **C5:** approximately 11–23 MHz analog bandwidth, using its own measured
  filter curve.
- **C61:** whole-MHz tuning from 2400–2500 MHz; up to 16,380 complex samples
  in IQ8/IQ10 at nominal 80/40/20/10/8/4 MS/s; approximately 13–54 MHz
  bandwidth. Divider rates follow the reference sensor firmware; independent
  RF/sample-rate calibration remains outstanding.
- **C6:** up to 16,380 complex samples in IQ8/IQ10 at nominal 80 MS/s;
  approximately 12–54 MHz bandwidth. Other rates and digital-filter modes
  remain unverified and are rejected. `TUNEEXT` allows whole-MHz tuning
  attempts from 2100–2800 MHz; standard Wi-Fi centers use calibrated tuning.
- **S2:** up to 12,284 complex samples in IQ8/IQ10 at nominal 80/40/16 MS/s;
  gain indices 0–82; approximately 15–60 MHz bandwidth; whole-MHz tuning
  attempts from 2212–2813 MHz.
- **S3:** approximately 13–69 MHz bandwidth using a separate measured curve.
- **S31:** up to 16,380 complex samples in IQ8/IQ10 at nominal
  80/40/20/10/8/4 MS/s; approximately 13–54 MHz bandwidth. The serial snapshot
  backend replaces the earlier vendor-USB/Ethernet image and does not advertise
  the old PARLIO 16 MS/s mode. See [S31 capture details](docs/s31-capture.md).

Extended tuning ranges are attempt ranges, not guarantees of PLL lock or
reception. The web viewer warns when tuning outside standard Wi-Fi centers.

The chip backends and shared serial transport live in `main/`. C5/C61/C6 share
`c5_c61_main.c` with chip-specific definitions. The S2-only compatibility
component in `platform/esp32s2/` fixes the pinned SDK's ROM USB descriptor
lifetime without changing other targets' SDK code.
