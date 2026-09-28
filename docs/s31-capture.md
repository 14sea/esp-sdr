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
PARLIO rate is intentionally absent from the handshake. Divider fields 4 and 6
measured approximately 13.333 and 8.889 MS/s, respectively, and are not exposed.
Old S31 firmware remains supported by the website through its own limits.

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

## Hardware validation, 2026-09-28

Test board: ESP32-S31 revision 0.0, UART bridge at `/dev/ttyUSB0`, 2 MBaud.
Native USB uses the shared transport but was not connected for this test.

99 captures passed payload CRC and firmware memory checks, covering six rates,
both precisions, lengths 256/257/4096/16380, gain controls, frequency endpoints,
and bandwidth controls. Invalid lengths/rates/gain/bandwidth/frequencies were
rejected. The 257-sample case exercises the packed IQ10 odd tail.

| Requested MS/s | Measured MS/s from duration slope |
| --- | --- |
| 80 | 79.77 |
| 40 | 40.01 |
| 20 | 20.01 |
| 10 | 10.00 |
| 8 | 8.00 |
| 4 | 4.00 |

Slopes compare 4,096 and 16,380 samples; timestamps are rounded to integer
microseconds. This is an acquisition-timing check, not an external RF-clock
calibration. A full 80 MS/s snapshot took about 208 us including settling.

The board reported manual gain indices 0–71. Gain changes altered captured
noise levels; indices are calibrated PHY table codes, not uniform dB steps.
At 2300 MHz and gain 71, median noise FFTs showed an outer/inner passband power
ratio of approximately -31 dB at 13 MHz bandwidth versus +1 dB at 54 MHz
(outer region 20–30 MHz, reference 2–5 MHz). This confirms filter response;
it is not a fresh precision calibration of every advertised MHz setting.

The actual ESP-WebSDR page was tested through a serial bridge to the hardware:
all six rates, IQ8/IQ10, gain/bandwidth changes, tuning and reconnect passed.
The viewer's existing capability negotiation needs no new chip-specific UI.
