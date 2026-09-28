# ESP32-S31 standard ADC capture

The S31 build uses `main/s31_main.c`, stock ESP-IDF PHY libraries and the
shared `burst_serial` transport. The former PARLIO/Ethernet/vendor-USB backend
is not part of the S31 image. No PSRAM or board peripherals are required;
the catalog profile uses a 2 MB flash layout and accepts larger flash chips.
The preview SDK remains pinned by `firmware-targets.json`.

## Protocol and controls

`INFO` reports `S31SDR 6 burst 16380`. `LIMITS?` advertises IQ8 and packed IQ10,
hardware sample rates 80/40/20/10/8/4 MS/s, the calibrated gain range, and an
approximate analog bandwidth range of 13–54 MHz. Hardware AGC is the default.
UART0 uses TX58/RX59 at 2 MBaud; USB Serial/JTAG uses the same protocol and
shared radio lease. Captures have gaps during serial transfer.

The raw dump words contain Q in bits 0–9 and I in bits 10–19. Firmware
normalizes them to the shared protocol's I-then-Q layout before packing.
There is no synthetic precision or software sample-rate conversion.

The standard dump path does not expose a verified 16 MS/s divider. The former
PARLIO rate is intentionally absent from the handshake. Only the rates
advertised by `LIMITS?` are supported.

## Memory handoff

The dump aperture starts at `0x2f060000`. A 26,624-word hardware ring fits below
ROM-owned memory. The lower 64 KiB guard at `0x2f050000` is reused for sample
normalization and in-place packing, avoiding a heap or PSRAM sample allocation.
Heap reservations and linker assertions keep application data, code and tasks
out of both regions. Four overrun canaries protect the end of the dump ring.

The `0xff000000` TCM dump gate is opened only with the peer CPU stalled through
ESP-IDF's IPC ISR and local interrupts masked. Register-only initialization
preserves the dump control register's reset fields (observed `0x03c00000`);
zeroing those fields stalls the writer. PHY calls occur with the gate closed.

Even a bounded snapshot must use the continuous-aperture control bit: the
vendor's finite test route can write at TCM offset zero, overlapping the app.
Each acquisition starts at zero, skips 256 settling samples, and polls the
write pointer until the requested span is complete. It stops before the first
wrap, leaving more than 9,000 samples of stop-latency margin at maximum size.
The writer and gate are shut down on both success and timeout, before memory
is read or either CPU resumes normal scheduling. Filter calibration is restored
before serial output, including capture failures. Maximum acquisition time is
bounded to 3,000,000 CPU cycles (9.375 ms at the configured 320 MHz).
