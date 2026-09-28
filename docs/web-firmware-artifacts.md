# Firmware artifacts for the website

`esp-sdr` builds firmware; `esp-web-sdr` consumes versioned manifests and
binaries. Neither build needs the other repository's source checkout.

## Export

After an ESP-IDF build, export one directory per chip/profile:

```sh
python3 tools/export_web_firmware.py \
  --build-dir build-s3 --board esp32s3 --label 'ESP32-S3 (2 MB or larger)' \
  --version "$FIRMWARE_COMMIT" --allow-larger-flash --output artifacts/esp32s3
```

The generic C5 and S3 builds need 2 MB flash and no board peripherals. They use
a fixed layout that is also valid on larger flash. `--allow-larger-flash`
explicitly advertises that compatibility. Omit it for profiles requiring the
exact configured size. Profiles with external wiring or memory requirements
must document those requirements in their labels/release documentation.

Use stable profile IDs such as `esp32c5` and `esp32s3`. The CLI retains the
`--board` name for compatibility with existing CI callers; it identifies a
firmware profile, not a PCB revision. The exporter reads target, flash settings
and offsets from ESP-IDF output, rejects diagnostic builds and invalid layouts,
and requires a new output directory to prevent mixing releases.

```text
artifact/
  manifest.json
  esp32s3/
    0-bootloader.bin
    1-partition-table.bin
    2-esp_sdr.bin
```

Schema version 1 has a `version` and `variants` object. Each variant includes
`revision` (profile ID), `label`, `target`, `chip` (esptool-js name), `version`,
`flash_size`, `flash_size_policy`, `flash_settings` and `parts`.
`flash_size_policy` is `exact` (also the default when omitted) or `minimum`.
Each part has `name`, numeric byte `offset`, `size`, `sha256` and `md5`.
All images must fit within the declared layout even when larger flash is allowed.

## Import and publish

From the website checkout:

```sh
python3 build_standalone.py
python3 flasher/build.py --artifacts /artifacts/esp32c5 /artifacts/esp32s3
```

These artifacts form the complete release set. The website verifies all sizes,
hashes and layouts, then embeds the binaries and manifests into `flash.html`.
Publish it alongside `spectrum.html`. Without `--artifacts`, the flasher rebuild
uses its checked-in release inputs. Firmware revision comes from the artifact,
never from the website's Git revision. Flash chip and capacity checks occur
before writing; the image's fixed layout is preserved on larger flash chips.

The artifact format supports additional chips, but flashing also requires a
compatible esptool-js target. Viewer support separately requires a compatible
I/Q transport driver. The included viewer currently supports C5/S3 protocol 6;
S31 and C61 do not acquire viewer support from the manifest alone.

This defines the handoff for a future CI pipeline; it does not configure one.
