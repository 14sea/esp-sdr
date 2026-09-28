# ESP-SDR firmware

Direct I/Q receive-only firmware for ESP32 chips. The original ESP32 and C5/C6/C61/S3/S31 backends
run without a display, status LED, buttons, or board-revision configuration.
They use internal RAM and require no PSRAM. Only the selected transport pins
are used. The browser viewer and installer live in [esp-web-sdr](../esp-web-sdr/README.md).

| Chip | Transport | Requirements / status |
| --- | --- | --- |
| ESP32 | UART0 only (GPIO1 TX / GPIO3 RX, 2 MBaud) | At least 2 MB flash; 2412–2472 MHz in 5 MHz steps; 80/40/16 MS/s snapshots |
| ESP32-C5 | Native USB Serial/JTAG | Generic firmware; at least 2 MB flash |
| ESP32-S3 | Native USB Serial/JTAG and optional UART0 | Generic firmware; at least 2 MB flash |
| ESP32-S31 | Native USB Serial/JTAG and optional UART0 | At least 2 MB flash; no PSRAM; 80/40/20/10/8/4 MS/s snapshots |
| ESP32-C6 | Native USB Serial/JTAG and optional UART0 | At least 2 MB flash; direct tuning attempts from 2100–2800 MHz |
| ESP32-C61 | Native USB Serial/JTAG and optional UART0 | Generic firmware; at least 2 MB flash; 2400–2500 MHz |

A generic board is not the same as a generic chip: each chip needs its own RF
backend. Other ESP32 models are not supported simply by selecting their IDF
target. C5/C6/C61/S3/S31 snapshots have capture gaps; RF sample rate is not sustained serial
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

All backends start with hardware AGC. `GAIN HARDWARE` releases forced gain;
`GAIN MANUAL <index>` selects a fixed PHY gain index. `GAIN AUTO` is no longer
accepted. S31 uses these same commands.

Both serial interfaces may be connected, but one client owns the shared radio.
Replies and binary frames return only on that client's port; other clients
receive `ERR busy`. `RELEASE` releases ownership. Five seconds of idle time also releases it. `CAPS SERIALLEASE` advertises ownership handling;
`CAPS DUALSERIAL` additionally indicates UART availability. `TRANSPORT?` returns
`TRANSPORT USB 0` or `TRANSPORT UART <baud>`. UART delivers fewer samples per
second than native USB; both carry framed snapshots, not gap-free acquisition.

## Other backends and releases

S31 uses `sdkconfig.defaults.esp32s31` with the preview SDK pinned in
`firmware-targets.json`. Build it with `tools/build_firmware.py --profile esp32s31`
and the version/output arguments below. Its standard ADC-dump backend needs
neither PSRAM nor Ethernet hardware. Native USB means USB Serial/JTAG; the
previous vendor-USB/Ethernet streaming image is replaced by serial snapshots.

`main/` contains the chip backends and shared serial transport.
`tools/export_web_firmware.py` exports built images for the website.
See [the artifact contract](docs/web-firmware-artifacts.md).
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
its submodules, run its `install.sh <target>` and source `export.sh`, then run (S31 no longer needs this repository's submodules):

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

C61 supports whole-MHz tuning from 2400 to 2500 MHz, IQ8/IQ10, gain and filter controls, and nominal 80/40/20/10/8/4 MS/s snapshot rates. These divider rates follow the reference sensor firmware; independent RF/sample-rate calibration remains outstanding. C61, S3 and S31 UART captures and controls have hardware coverage; C5 captures and controls were also tested over native USB Serial/JTAG.

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
C5 advertises an approximate 11–23 MHz receive bandwidth and applies its own
measured capacitor-code curve to BBTOP registers 6/7. ESP-WebSDR defaults to
20 MHz and 80 MS/s.

S31 exposes the same serial handshake at 2 MBaud on UART0 TX58/RX59 and
native USB Serial/JTAG. `main/s31_main.c` captures up to 16,380 contiguous
complex samples from the standard ADC dump engine, in IQ8 or packed IQ10.
Hardware rates are 80/40/20/10/8/4 MS/s. The old PARLIO-specific 16 MS/s mode
is not advertised by this backend. Gain, bandwidth and rate limits are
negotiated through `LIMITS?`, so ESP-WebSDR enables the new modes automatically.
See [S31 capture details](docs/s31-capture.md) for the memory handoff and tests.

## ESP32-C6

Build with `IDF_TARGET=esp32c6` and `sdkconfig.defaults.esp32c6`, using a separate build directory and sdkconfig. UART0 uses GPIO16 (TX), GPIO17 (RX), 2 Mbaud, 8N1; boot messages use 115200 baud. The shared C5/C61 receiver also handles C6, with hardware differences in `main/c6_chip.h`.

C6 supports IQ8/IQ10 snapshots of up to 16380 complex samples at nominal 80 MS/s, hardware AGC and manual gain. Whole-MHz tuning requests from 2100–2800 MHz are attempted using the direct PLL path. Standard Wi-Fi channel centers retain the calibrated channel path; other frequencies show a warning in web-sdr because PLL lock and reception are not guaranteed. `CAPS` advertises `TUNEEXT` and `RANGE?` reports the attempt range. The analog bandwidth is adjustable over approximately 12–54 MHz. Other sample rates and digital-filter modes are not verified and are rejected. `LIMITS?` advertises these limits to the web app.

The C6 dump uses a reserved 128 KiB SRAM bank at 0x40840000, bounded completion polling and four overrun canaries. Hardware testing on C6 revision 0.1 used UART; native USB and independent RF calibration remain untested.
