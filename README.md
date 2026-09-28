# ESP-SDR firmware

Direct I/Q receive-only firmware for ESP32 chips. The C5/S3 backends
run without a display, status LED, buttons, or board-revision configuration.
They use internal RAM and require no PSRAM. Only the selected transport pins
are used. The browser viewer and installer live in [esp-web-sdr](../esp-web-sdr/README.md).

| Chip | Transport | Requirements / status |
| --- | --- | --- |
| ESP32-C5 | Native USB Serial/JTAG | Generic firmware; at least 2 MB flash |
| ESP32-S3 | Native USB Serial/JTAG and optional UART0 | Generic firmware; at least 2 MB flash |
| ESP32-S31 | Existing Ethernet/native high-speed USB streaming backend | Requires the existing SDK, memory and transport hardware configuration |
| ESP32-C61 | Unfinished shared/open-PHY development path | Not currently buildable; excluded from release artifacts |

A generic board is not the same as a generic chip: each chip needs its own RF
backend. Other ESP32 models are not supported simply by selecting their IDF
target. C5/S3 snapshots have capture gaps; RF sample rate is not sustained serial
throughput. Gain and power measurements are uncalibrated.

## Build C5 or S3

Activate a compatible ESP-IDF environment, then choose the chip explicitly and
use a separate build directory/configuration:

```sh
idf.py -B build-s3 -DIDF_TARGET=esp32s3 \
  -DSDKCONFIG=sdkconfig.s3 \
  -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32s3 build
idf.py -B build-s3 -p /dev/ttyACM0 flash
```

For C5, substitute `c5` / `esp32c5` in the paths and target above. The standard
`idf.py set-target esp32s3` workflow also loads the target-specific defaults;
shared defaults do not enable another chip’s memory or peripherals. No PHY
submodules are needed for these two backends. Both defaults use a 2 MB flash
layout with a single application partition. These images also work on larger
flash devices, using only that layout; they do not resize partitions to consume
the extra flash. For a different layout, use a separate build configuration and
export its actual flash requirements.

Connect native USB to the chip's USB Serial/JTAG pins. S3 also accepts the same
commands and I/Q payloads on UART0: **2000000 baud, 8N1, no flow control**, TX
GPIO43 / RX GPIO44 by default. Connect an appropriate 3.3 V USB-to-UART bridge
with crossed TX/RX and common ground, or use the board's existing bridge.
`idf.py menuconfig` exposes `ESP_SDR_S3_UART_ENABLED`, `ESP_SDR_S3_UART_BAUD`,
`ESP_SDR_S3_UART_TX_PIN` and `ESP_SDR_S3_UART_RX_PIN`. Disable UART to leave
those pins unused by the application. Native USB ignores the host baud setting.

C5/S3 start with hardware AGC. `GAIN HARDWARE` releases forced gain;
`GAIN MANUAL <index>` selects a fixed PHY gain index. `GAIN AUTO` is no longer
accepted. The S31 control API uses gain mode 0 for hardware AGC, 1 for manual
and 2 for expert gain; its software gain controller and telemetry are removed.

Both S3 interfaces may be connected, but one client owns the shared radio.
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
pins an ESP-IDF commit and selects its chip defaults. C5, S3 and S31 build in
independent jobs; a failed build never becomes a firmware artifact. C61 is
listed as unavailable because its current shared backend requires S31-specific
Ethernet/USB components. Adding a supported profile to the catalog automatically
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
