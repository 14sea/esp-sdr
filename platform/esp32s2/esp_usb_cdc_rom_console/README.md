# S2 ROM CDC compatibility component

Copied from ESP-IDF commit 25fe69f946311abdaf9ad56591f25fedbc20ac98.
Only ESP32-S2 selects this override. It differs from the SDK in two ways:
`rom_usb_cdc_set_descriptor_patch()` is called during ROM cleanup, so descriptors
point into application memory rather than reclaimed bootloader RAM, and
`esp_usb_console_write_direct()` lets ESP-SDR queue more than one USB packet per
write. The register accesses use `soc/usb_dwc_struct.h` so the copy builds with
current ESP-IDF.
See https://github.com/espressif/esp-idf/issues/18841. Preserve the upstream
licenses. Reassess the override when updating the S2 SDK pin.
