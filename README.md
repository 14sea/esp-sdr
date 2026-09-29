# ESP-SDR firmware

<img src="docs/espargos-logo.png" width="40%" align="right" alt="ESPARGOS logo">

Receive-only I/Q snapshot firmware for ESP32 chips. Uses internal RAM; no PSRAM
or extra peripherals required.

With the help of LLMs, we discovered an undocumented feature in Espressif's
ESP32 chips that bypasses the fixed-function modems to capture raw IQ baseband
samples. ESP-SDR uses this feature to turn supported chips into low-cost
software-defined radio receivers for the 2.4 GHz band (and the 5 GHz band on
the ESP32-C5).

[Project overview](https://espargos.net/espsdr/) ·
[Browser SDR viewer](https://espargos.net/espsdr/app/) ·
[Browser firmware installer](https://espargos.net/espsdr/app/flash.html)

**Parts of the firmware code are AI-generated.**
While we have a very good understanding of how the IQ sampling functionality works on the ESP32-C61 chip (used in our ESPARGOS One array), making IQ sampling work on the whole range of ESP32 family chips would have been too much work without LLM support.

## Chip support

| Chip | Status | Native USB | UART0 TX / RX | Minimum flash |
| --- | --- | --- | --- | --- |
| ESP32 | ✅ Supported | — | GPIO1 / GPIO3 | 2 MB |
| ESP32-C2 | 🚧 Unsupported | — | — | — |
| ESP32-C3 | ✅ Supported | Serial/JTAG | GPIO21 / GPIO20 | 2 MB |
| ESP32-C5 | ✅ Supported | Serial/JTAG | GPIO11 / GPIO12 | 2 MB |
| ESP32-C6 | ✅ Supported | Serial/JTAG | GPIO16 / GPIO17 | 2 MB |
| ESP32-C61 | ✅ Supported | Serial/JTAG | GPIO11 / GPIO10 | 2 MB |
| ESP32-H2 | 🚧 Unsupported | — | — | — |
| ESP32-H21 | 🚧 Unsupported | — | — | — |
| ESP32-H4 | 🚧 Unsupported | — | — | — |
| ESP32-P4 | ❌ Unsupported; no integrated radio | — | — | — |
| ESP32-S2 | ✅ Supported | USB-OTG CDC | GPIO43 / GPIO44 | 4 MB |
| ESP32-S3 | ✅ Supported | Serial/JTAG | GPIO43 / GPIO44 | 2 MB |
| ESP32-S31 | ✅ Supported | Serial/JTAG | GPIO58 / GPIO59 | 2 MB |

## Connect

Use native USB or a 3.3 V USB-to-UART adapter with crossed TX/RX and common
ground. UART defaults to **2,000,000 baud, 8N1, no flow control** on the pins
above. Native USB ignores the host baud setting.

Configure UART with the `ESP_SDR_UART_*` menuconfig options.
C5 UART requires a firmware build with UART support; older USB-only images
must be reflashed. See [C5 debugging results](docs/c5-debug.md).
S2 UART is build-verified but untested with an adapter; reflash older USB-only
images to enable it.

One client controls the radio at a time. `RELEASE` or five seconds of idle time
releases it; other clients receive `ERR busy`.

## Build and flash

[firmware-targets.json](firmware-targets.json) lists the supported profiles and
pins their ESP-IDF commits, including the preview SDK for S31. Check out the
matching SDK, initialize its submodules, run `install.sh <target>`, and source
`export.sh`.

Use a separate build directory and configuration for each chip:

```sh
idf.py -B build-s3 -DIDF_TARGET=esp32s3 \
  -DSDKCONFIG=sdkconfig.s3 \
  -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32s3 build
idf.py -B build-s3 -p /dev/ttyACM0 flash
```

Substitute the target and paths for your chip. S31 also requires `idf.py --preview`.

## Source layout

- `main/targets/<target>/`: chip receiver or adapter, tuning helpers, and the
  linker guard for its capture SRAM. CMake selects only the requested target.
- `main/families/c5_c6_c61/`: receiver shared by C5, C6, and C61; its `chip.h`
  comes from the selected target directory.
- `main/common/`: burst serial transport, gain control, limits, and bandwidth
  helpers. The gain-table wrapper is linked only for C61 and S31.
- `main/diagnostics/`: optional register probes, excluded from release exports.
- `platform/esp32s2/`: pinned ROM USB CDC compatibility component.

The application component and UART configuration stay in `main/`. Target SDK
defaults stay at the repository root for the build tools and ESP-IDF defaults
lookup. The firmware uses the burst protocol over UART/native USB; the former
Ethernet and vendor USB streaming application is no longer included.

Run `python3 -m unittest discover -s tests` for host checks. Build every profile
with `tools/build_firmware.py` and its pinned SDK before distributing a change;
the CI matrix does this automatically. Preserve the target SRAM guards and
gain-table linker wrappers when moving or refactoring receiver code.

## Receive controls

Hardware AGC is the default. `GAIN MANUAL <index>` sets manual gain;
`GAIN HARDWARE` restores AGC. `LIMITS?` reports available gain indices,
bandwidths, sample rates and bit depths. `BANDWIDTH <MHz>` sets approximate
analog bandwidth; zero selects the widest setting.

All eight chips accept tuning attempts from **100–6000 MHz in 1 MHz steps**.
The viewer shows an informational warning outside 2400–2483.5 MHz, with
5150–5895 MHz also treated as the supported 5 GHz Wi-Fi band on C5. The warning never blocks tuning.
These are software attempt limits; the expanded range has not been hardware
validated.

- **ESP32:** 80/40/16 MS/s.
- **C3:** 80 MS/s; 14–62 MHz analog bandwidth.
- **C5:** 11–48 MHz bandwidth; selects its 5 GHz RF path above 3000 MHz.
- **C61:** 80/40/20/10/8/4 MS/s; 13–54 MHz bandwidth.
- **C6:** 80 MS/s; 12–54 MHz bandwidth.
- **S2:** 80/40/16 MS/s; 15–60 MHz bandwidth; up to 12,284 complex samples.
- **S3:** 13–69 MHz bandwidth.
- **S31:** 80/40/20/10/8/4 MS/s; 13–54 MHz bandwidth.

Captures have gaps; nominal sample rates exceed sustained serial throughput.
Gain and power are uncalibrated. Extended tuning does not guarantee PLL lock
or reception; the viewer uses the ISM-band warning described above.

See [receive-control details](docs/rx-controls.md).
