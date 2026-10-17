# How the capture works

Espressif doesn't document any of this. Everything below comes from reading
the RF test library that ships with ESP-IDF (`librftest.a`, function
`adctrig`), from disassembly, and from trying things on real boards. Register
names here are our own.

## The idea in one picture

```
antenna → LNA/mixer → filter → ADC ──→ Wi-Fi / Bluetooth modem (normal path)
                                 │
                                 └──→ "dump" engine ──→ SRAM bank ──→ CPU → USB/UART
```

Every supported chip contains a small debug block, which we call the **dump
engine**. Espressif uses it in factory RF testing to record what the receiver
sees. When it runs, it copies baseband samples from the receiver into a block
of ordinary on-chip SRAM, one 32-bit word per sample. Once it's done, the CPU
reads that memory like any other array and sends it to the host.

The radio still has to be switched on and tuned first. ESP-SDR does that with
the normal Wi-Fi (or, on the H2, Bluetooth) initialization, then uses PHY
library calls to pick a frequency and disable transmit. After that, the modem
is simply ignored.

## One snapshot, step by step

The ESP32-C3 is the simplest example (`main/targets/esp32c3/receiver.c`). The
other chips follow the same pattern with different addresses; see the table at
the end.

**1. Keep the heap out of the capture memory.** The dump engine writes into a
fixed SRAM bank (at `0x3fcb0000` on the C3). If FreeRTOS put a task stack
or a Wi-Fi buffer there, the capture would overwrite it. Both of these keep that
from happening:

```c
SOC_RESERVE_MEMORY_REGION(0x3fca0000, 0x3fcc0000, c3_rf_dump);
```

plus a linker script (`sram_guard.ld`) so no static data ends up there either.

**2. Fill the buffer with a marker.** Every word gets `0xa5a0055a`. Afterwards,
any word still holding the marker was never written, so a truncated capture is
detected instead of silently returning stale data.

**3. Hand the SRAM bank to the dump engine.** SRAM banks have an owner. A
register in the system block (`0x600c1020` on the C3, "SRAM owner" in the code)
decides whether the CPU or the dump engine may write a bank:

```c
REG_WRITE(SRAM_OWNER_REG, (owner & ~7u) | 2u | 8u);   // block 0x3fca0000, upper half
```

On the C3, bits 0–2 each give one 128 KiB block to the engine (bit 0:
`0x3fc80000`, bit 1: `0x3fca0000`, bit 2: `0x3fcc0000`), and bit 3 makes it
write the upper 64 KiB half, so `2 | 8` means `0x3fcb0000`. The CPU loses the
whole block, not just the half being written. While the engine runs, reads
anywhere in it return garbage and writes are lost. Getting a bit wrong is
dramatic: bit 0 or bit 2 makes the engine overwrite the program's own memory or
stack.

While the dump engine owns the bank, the CPU must not read it. Some chips return
garbage (the C3 returns a repeating four-word pattern), so the firmware does all
of this inside a critical section.

**4. Choose what to record.** The lane register (`0x60033d90` on the C3) has four
6-bit fields. Each field selects which internal debug signal is recorded. The
values 0, 1, 2, 3 select the receiver's I and Q outputs:

```c
REG_WRITE(0x60033d90, (0 << 0) | (1 << 6) | (2 << 12) | (3 << 18));
```

Other values give other internal signals (AGC state, demodulator internals). We
have only characterized the I/Q selection.

**5. Start the dump.** One control register (`0x60033d5c` on the C3) does everything:

| Bits | Meaning |
| --- | --- |
| 0–14 | Number of words (samples) to record |
| 15–16 | Clock divider, on chips that support several sample rates (S2, S3, C2, C3, C6, H2) |
| 18 | Done flag, set by hardware |
| 19 | Trigger: write 1 then 0 to start |
| 20+ | Data source and mode, on the older ESP32, S2 and S3 (source 0 is I/Q) |
| 31 | Enable |

```c
uint32_t ctrl = 0x80000000u | n;            // enable + sample count
REG_WRITE(0x60033d5c, ctrl);
REG_WRITE(0x60033d5c, ctrl | (1u << 19));   // trigger...
REG_WRITE(0x60033d5c, ctrl);                // ...as a pulse
while (!(REG_READ(0x60033d5c) & (1u << 18))) {}   // wait for done (with a timeout)
```

At 80 MS/s, a full 16,380-sample capture takes about 205 µs.

**6. Put everything back.** Clear the control register, restore the SRAM owner
and any other registers that were changed, and leave the critical section. The
CPU can now read the samples.

## What a captured word looks like

```
 31        28 27                20 19                10 9                 0
┌────────────┬────────────────────┬────────────────────┬────────────────────┐
│  (unused)  │  gain index (8 b)  │   Q (10-bit signed) │   I (10-bit signed) │
└────────────┴────────────────────┴────────────────────┴────────────────────┘
```

- **I and Q** are two's complement 10-bit values. `CAP16` sends only their
  upper 8 bits, `CAP20` packs the full 20 bits, and `CAP` sends the raw word.
- **Bits 20–27** hold the receiver gain index that was active for that sample.
  The firmware copies it into spectrum frame headers so hosts can see what the
  AGC did.

The I/Q values come straight from the ADC side of the receiver: no Wi-Fi
filtering or demodulation has happened yet. That is why ESP-SDR can see any
signal in the receiver's passband, not just Wi-Fi.

## Continuous capture (ring mode)

A snapshot has to stop the receiver so the CPU can read the bank. For
continuous spectra (C6, C61, S3, S31 over native USB) and the S3's continuous
I/Q stream, the dump engine runs in **circular mode** instead
(`main/common/ring_capture.c`):

- A control bit (`DUMP_CTRL_CIRCULAR`) makes the engine wrap around a
  16,384-sample ring instead of stopping.
- A **write-index register** (`DUMP_WRITE_INDEX_REG`) tells the CPU where the
  engine is currently writing.
- The ring spans several SRAM banks. While the engine fills one bank, the
  firmware takes ownership of the bank it has just finished, processes it
  (FFT, filtering), and hands it back before the engine comes around again.

The banks must lie in blocks the CPU can still read while the engine writes the
other one. That is why the C3 has no continuous mode: its owner bits work on
128 KiB blocks, and the only blocks besides the one already reserved hold the
program and the heap.

This is a race against the hardware: if processing is too slow, the engine
overwrites data that hasn't been read yet. The firmware fills unread areas with
a second marker word (`0xa5c33c5a`) and checks for it, so it can report gaps
instead of delivering corrupt data. On the S3, the second CPU core does the
signal processing while the first core manages banks and USB.

## The ESP32-S31 streaming firmware

The S31's streaming firmware doesn't use SRAM banks at all. The receiver's
internal debug bus is routed through the **GPIO matrix**, an on-chip crossbar
that can connect internal signals to GPIO pads, and looped back into the
**PARLIO** peripheral (a parallel-input DMA engine). PARLIO streams 16 bits per
sample (8-bit I, 8-bit Q) into memory continuously with DMA, with no CPU
involvement and no gaps. See [s31-streaming.md](s31-streaming.md).

## Per-chip registers

| Chip | Capture memory | SRAM owner register | Dump control | Lane select |
| --- | --- | --- | --- | --- |
| ESP32 | `0x3ffe8000` (64 KiB) | DPORT `MAC_DUMP_MODE` = 3 | `0x60033d90` | `0x60033dc4` (bytes 0,1,2,3) |
| ESP32-S2 | `0x3fff0000` (48 KiB) | `0x3f4c10dc` | `0x60033d64` | `0x60033d98` |
| ESP32-S3 | `0x3fcd0000` (snapshot) | `0x600c101c` | `0x60033d5c` | `0x60033d90` |
| ESP32-C2 | `0x3fcc0000` (32 KiB) | `0x600c1018` | `0x6004bc04` | `0x6004bc14` |
| ESP32-C3 | `0x3fcb0000` (64 KiB) | `0x600c1020` | `0x60033d5c` | `0x60033d90` |
| ESP32-C5 | `0x40830000` | `0x60095004` | `0x600a9004` | `0x600a9018` |
| ESP32-C6 | `0x40840000` | `0x60095004` | `0x600a9004` | `0x600a9014` |
| ESP32-C61 | `0x40830000` | `0x60095004` | `0x600a9004` | `0x600a9018` |
| ESP32-H2 | `0x40820000` (64 KiB) | `0x60095004` | `0x600a4c04` | `0x600a4c14` |
| ESP32-S31 (burst) | `0x2f040000` | — | `0x20109004` | — |

Some chips need a few more writes, such as clock enables, a trigger-source bit or
a "dump mode" register. Those are listed in the chip notes
([C2](esp32c2.md), [H2](esp32h2.md)) and commented in each `receiver.c`.

## Things we don't know

- What most other lane-select values record.
- Whether the gain index in bits 20–27 is exact on every chip, or only on the
  ones where we compared it with the gain we set.
- Why the C3 returns a repeating pattern when the CPU reads a bank the dump
  engine owns.
- Whether the dump clock dividers apply any filtering. On the S2 and H2 they
  appear to simply skip samples, so lower rates alias.
