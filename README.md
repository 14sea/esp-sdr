# ESP-SDR firmware

Direct I/Q receive-only firmware for ESP32 chips. The original ESP32 and C5/C6/C61/S3 backends
run without a display, status LED, buttons, or board-revision configuration.
They use internal RAM and require no PSRAM. Only the selected transport pins
are used. The browser viewer and installer live in [esp-web-sdr](../esp-web-sdr/README.md).

| Chip | Transport | Requirements / status |
| --- | --- | --- |
| ESP32 | UART0 only (GPIO1 TX / GPIO3 RX, 2 MBaud) | At least 2 MB flash; 2412–2472 MHz in 5 MHz steps; 80/40/16 MS/s snapshots |
| ESP32-C5 | Native USB Serial/JTAG | Generic firmware; at least 2 MB flash |
| ESP32-S3 | Native USB Serial/JTAG and optional UART0 | Generic firmware; at least 2 MB flash |
| ESP32-S31 | Serial snapshots on native USB Serial/JTAG/UART0, plus Ethernet/native high-speed USB streaming | Requires the existing SDK, memory and transport hardware configuration |
| ESP32-C6 | Native USB Serial/JTAG and optional UART0 | At least 2 MB flash; direct tuning attempts from 2100–2800 MHz |
| ESP32-C61 | Native USB Serial/JTAG and optional UART0 | Generic firmware; at least 2 MB flash; 2400–2500 MHz |

A generic board is not the same as a generic chip: each chip needs its own RF
backend. Other ESP32 models are not supported simply by selecting their IDF
target. C5/C6/C61/S3 snapshots have capture gaps; RF sample rate is not sustained serial
throughput. Gain and power measurements are uncalibrated.

## Build ESP32, C5, C6, C61 or S3

Activate a compatible ESP-IDF environment, then choose the chip explicitly and
use a separate build directory/configuration:

```sh
idf.py -B build-s3 -DIDF_TARGET=esp32s3 \
  -DSDKCONFIG=sdkconfig.s3 \
  -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32s3 build
idf.py -B build-s3 -p /dev/ttyACM0 flash
```

For the original ESP32, use `esp32` as the target and `sdkconfig.defaults.esp32`.
For C5, C6 or C61, substitute `c5` / `esp32c5`, `c6` / `esp32c6` or `c61` / `esp32c61` in the paths and target above. The standard
`idf.py set-target esp32s3` workflow also loads the target-specific defaults;
shared defaults do not enable another chip’s memory or peripherals. No PHY
submodules are needed for these five backends. All five defaults use a 2 MB flash
layout with a single application partition. These images also work on larger
flash devices, using only that layout; they do not resize partitions to consume
the extra flash. For a different layout, use a separate build configuration and
export its actual flash requirements.

Connect native USB to the chip's USB Serial/JTAG pins. S3 also accepts the same
commands and I/Q payloads on UART0: **2000000 baud, 8N1, no flow control**, TX
GPIO43 / RX GPIO44 by default. Connect an appropriate 3.3 V USB-to-UART bridge
with crossed TX/RX and common ground, or use the board's existing bridge.
`idf.py menuconfig` exposes `ESP_SDR_UART_ENABLED`, `ESP_SDR_UART_BAUD`,
`ESP_SDR_UART_TX_PIN` and `ESP_SDR_UART_RX_PIN`. Disable UART to leave
those pins unused by the application. Native USB ignores the host baud setting.

All four backends start with hardware AGC. `GAIN HARDWARE` releases forced gain;
`GAIN MANUAL <index>` selects a fixed PHY gain index. `GAIN AUTO` is no longer
accepted. The S31 control API uses gain mode 0 for hardware AGC, 1 for manual
and 2 for expert gain; its software gain controller and telemetry are removed.

Both serial interfaces may be connected, but one client owns the shared radio.
Replies and binary frames return only on that client's port; other clients
receive `ERR busy`. `RELEASE` releases ownership. Five seconds of idle time also releases it. `CAPS SERIALLEASE` advertises ownership handling;
`CAPS DUALSERIAL` additionally indicates UART availability. `TRANSPORT?` returns
`TRANSPORT USB 0` or `TRANSPORT UART <baud>`. UART delivers fewer samples per
second than native USB; both carry framed snapshots, not gap-free acquisition.

## Other backends and releases

S31 uses `sdkconfig.defaults.esp32s31` and its existing PSRAM and transport configuration.
Its Ethernet mode needs the configured external PHY and wiring. This backend
has not been converted into a hardware-independent configuration. Initialize
its dependencies with `git submodule update --init --recursive`, and enable the
component manager when downloading registry components. The custom TinyUSB
component uses `managed_components/espressif__tinyusb`.

`main/` contains chip backends, transports and the embedded S31 network status
page. `components/` holds dependencies; `tools/export_web_firmware.py` exports
built images for the website. See [the artifact contract](docs/web-firmware-artifacts.md).
Do not enable diagnostic probe options in release builds.

## CI firmware builds

[Build firmware](.github/workflows/firmware.yml) runs on pushes, pull requests
and manual dispatch. `firmware-targets.json` is the build catalog: each profile
pins an ESP-IDF commit and selects its chip defaults. C5, C6, C61, S3 and S31 build in
independent jobs; a failed build never becomes a firmware artifact. Adding a supported profile to the catalog automatically
adds it to CI; it also needs working source/defaults and a compatible SDK.

Download `esp-sdr-firmware` from the workflow's artifacts for the complete set,
or `firmware-<profile>` for a single profile. The combined set is produced only
when every build succeeds. It is retained for 90 days; individual builds for 30.
Artifacts include images, checksums, flash offsets, source/SDK revisions and
`flash_args` for command-line flashing. The combined artifact has one root
`manifest.json` and is ready to deploy as the static website’s `firmware/` folder. These are build artifacts, not hardware
qualification results or automatically published GitHub Releases.

To reproduce a CI profile, check out the SDK commit in the catalog, initialize
its submodules, run its `install.sh <target>` and source `export.sh`. Initialize
this repository's submodules for S31, then run:

```sh
python tools/build_firmware.py --profile esp32s3 --version "$(git rev-parse HEAD)" \
  --build-root /opt/codex-scratch-space/esp-sdr-builds \
  --output /opt/codex-scratch-space/esp-sdr-artifacts
```

Use a fresh output directory for each release. The build helper checks the SDK
commit, keeps chip configurations separate and disables diagnostic probes.
Builds default to two compiler jobs to limit RAM usage; adjust with `--jobs`.
`--build-root` keeps large build files on disk instead of a RAM-backed `/tmp`.
See [the handoff instructions](docs/web-firmware-artifacts.md) for website imports.

## ESP32-C61

Build with `IDF_TARGET=esp32c61` and `sdkconfig.defaults.esp32c61`, using a separate build directory and sdkconfig as above. UART0 uses GPIO11 (TX), GPIO10 (RX), 2 Mbaud, 8N1. Boot messages use 115200 baud. Native USB Serial/JTAG uses the same burst protocol.

C5 and C61 share `main/c5_c61_main.c`; chip-specific capture and tuning limits live in `main/c5_c61_chip.h`. The serial transport is shared with S3. Old S3 UART configuration names migrate through `main/sdkconfig.rename`. C61 reserves SRAM bank 3 for 16380 complex samples and four overrun canaries, leaving ROM memory accessible during capture.

C61 supports whole-MHz tuning from 2400 to 2500 MHz, IQ8/IQ10, gain and filter controls, and nominal 80/40/20/10/8/4 MS/s snapshot rates. These divider rates follow the reference sensor firmware; independent RF/sample-rate calibration remains outstanding. C61, S3 and S31 UART captures and controls have hardware coverage; C5 has build and host-test coverage.

## Negotiated receive controls

Protocol 6 now advertises `RXLIMITS` in `CAPS`. Query `LIMITS?` after `INFO`:

```text
LIMITS {"gain":[0,76,1],"bandwidth":[13,54,1,0],"rates":[80000000,40000000,20000000,10000000,8000000,4000000],"bits":[8,10]}
```

`gain` is `[minimum, maximum, step]` in PHY table indices. The maximum is
read from the calibrated table at runtime; it is not a fixed limit of 50/55.
Observed maxima on the connected boards were C61 76, S3 82 and S31 71.
`bandwidth` is `[minimum, maximum, step, default]` in MHz, or `null` when no
verified MHz calibration exists. Zero means open/widest. `BANDWIDTH <MHz>`
rejects out-of-range values; it never silently clamps them.

C61 uses the `sensor-firmware/main/iq/modem.c` 13–54 MHz interpolation and
BBTOP registers 4/5. It mirrors forced gain entries into the second table and
disables RF saturation intervention for manual gain; hardware AGC restores
its calibrated settings. S31 retains its own 13–54 MHz mapping. S3 uses a
separate measured 13–69 MHz curve. See [calibration notes](docs/rx-controls.md).
C5 advertises `bandwidth: null` and uses the PHY's automatic filter.

S31 exposes the same serial handshake at 2 MBaud on UART0 TX58/RX59 and
native USB Serial/JTAG. It collects 16,384 contiguous complex samples from
the existing PARLIO receiver into a 32 KiB PSRAM snapshot, stops acquisition,
then sends the CRC-framed response. `CAP16` supports rate codes 6/4/5
(16/8/4 MS/s); it does not advertise synthetic 10-bit precision. Other
serial clients receive `ERR busy` until release or expiry. A serial request
also rejects an already active Ethernet/vendor-USB stream. A later stream
start on another transport can replace the serial capture, which reports an
error rather than returning mixed data.

## ESP32-C6

Build with `IDF_TARGET=esp32c6` and `sdkconfig.defaults.esp32c6`, using a separate build directory and sdkconfig. UART0 uses GPIO16 (TX), GPIO17 (RX), 2 Mbaud, 8N1; boot messages use 115200 baud. The shared C5/C61 receiver also handles C6, with hardware differences in `main/c6_chip.h`.

C6 supports IQ8/IQ10 snapshots of up to 16380 complex samples at nominal 80 MS/s, hardware AGC and manual gain. Whole-MHz tuning requests from 2100–2800 MHz are attempted using the direct PLL path. Standard Wi-Fi channel centers retain the calibrated channel path; other frequencies show a warning in web-sdr because PLL lock and reception are not guaranteed. `CAPS` advertises `TUNEEXT` and `RANGE?` reports the attempt range. The analog bandwidth is adjustable over approximately 12–54 MHz. Other sample rates and digital-filter modes are not verified and are rejected. `LIMITS?` advertises these limits to the web app.

The C6 dump uses a reserved 128 KiB SRAM bank at 0x40840000, bounded completion polling and four overrun canaries. Hardware testing on C6 revision 0.1 used UART; native USB and independent RF calibration remain untested.
