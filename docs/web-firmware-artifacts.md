# Firmware artifacts for a static website

Firmware CI builds and packages the binaries. ESP-WebSDR serves ordinary static
files and loads the firmware catalog at runtime. **There is no website build.**

## Per-profile export

`tools/build_firmware.py` builds a catalog profile and invokes the exporter.
For an existing ESP-IDF build, export it directly:

```sh
python3 tools/export_web_firmware.py \
  --build-dir build-s3 --board esp32s3 --label 'ESP32-S3 (2 MB or larger)' \
  --version "$FIRMWARE_COMMIT" --allow-larger-flash --output artifacts/esp32s3
```

Each profile artifact contains its manifest, image directory, `flash_args` and
`flash_command.txt`. The build helper also records the SDK revision in
`build-info.json`. The exporter rejects diagnostic builds and invalid layouts.
Flash-size detection is resolved to the concrete configured capacity. A new
output folder is required to avoid mixing releases.

Original ESP32 and C5/C6/C61/S3 profiles use a 2 MB layout, no PSRAM or board peripherals.
`--allow-larger-flash` advertises compatibility with larger physical flash.
Omit it for profiles requiring an exact size. Hardware-specific profiles such
as S31 retain their requirements in their labels and documentation.

## Combined CI artifact

After all matrix builds succeed, firmware CI collects the exported profiles:

```sh
python3 tools/collect_firmware.py --input downloaded-firmware \
  --output firmware --catalog firmware-targets.json
```

The collector validates every profile, size, checksum, image range and path,
rejects duplicates, and requires the complete catalog set. It copies files
without buffering the full release in RAM. The combined `esp-sdr-firmware`
artifact has this layout:

```text
manifest.json
esp32c5/
  0-bootloader.bin
  1-partition-table.bin
  2-esp_sdr.bin
  build-info.json
  flash_args
  flash_command.txt
esp32c6/...
esp32c61/...
esp32s3/...
esp32s31/...
```

Download/extract this artifact and deploy its contents as `esp-web-sdr/firmware/`.
The website fetches `firmware/manifest.json` and constructs its options directly.
No website scripts, SDK, or firmware source are needed for this deployment.
Publish the complete folder together, preferably atomically; inconsistent files
are rejected by the browser's integrity checks before flashing.

## Manifest contract

Schema version 1 contains `version` and a nonempty `variants` object keyed by
stable profile IDs. Each variant includes `label`, `target`, `chip` (esptool-js
name), `version`, `flash_size`, `flash_size_policy`, `flash_settings` and `parts`.
The policy is `exact` by default or `minimum` to allow larger physical flash.
Each image has a basename `name`, numeric byte `offset`, `size`, `sha256` and
`md5`. Its URL is `<profile>/<name>` relative to the manifest. No binaries are
embedded in JSON or HTML. The browser downloads only a selected profile when
installing, and validates all its images before writing.

Firmware availability is independent of browser loader/viewer support. The
website bundles esptool-js 0.7.0, which supports all current profiles,
including S31. For command-line flashing use `<profile>/flash_command.txt`,
replacing `PORT`. The S31 artifact retains `--no-stub` for SDK compatibility.
All profiles expose the viewer's serial snapshot protocol; S31 also
retains its Ethernet/vendor-USB streaming transports.

The website and firmware workflows produce artifacts without publishing to a
server. A later deployment job only needs to copy static website files and the
complete firmware folder; it needs no website build step.
