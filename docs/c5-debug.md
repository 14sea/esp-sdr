# ESP32-C5 UART and bandwidth fixes (2026-09-29)

Both issues were firmware configuration problems. UART was compiled out for
C5, and receive setup always calibrated the PHY in its narrow channel mode.
The fixed firmware supports both serial interfaces and advertises an
approximate 11–48 MHz receive-bandwidth range.

## Bandwidth root cause

ESP32-C5 supports HT20 and HT40, as documented by
[Espressif](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c5/api-guides/wifi-driver/wifi-mac-protocols.html).
Our shared receiver nevertheless always called `phy_chip_set_chan(..., 0)`.
The C5 tuning helper passed that mode to `phy_set_chanfreq` when calibrating
on 2412 or 5180 MHz, then programmed the requested PLL frequency.

In the pinned C5 PHY, channel setup does more than set a digital bandwidth
field. It configures ADC/filter settings, calls `phy_set_chan_reg`, configures
baseband channel width, rebuilds PBUS analog-control tables through
`phy_set_pbus_mem`, and updates calibration state. Changing individual
registers after a mode-0 calibration did not reproduce this transition. This
is why the initial register sweeps missed the wider path.

The complete mode-1 transition opens a wider path while keeping BBTOP RX0
capacitor registers 6/7 effective. Modes 2/3 select a different analog branch;
changing 6/7 had no useful bandwidth effect there. The implementation uses
mode 1 for wide SDR reception, rather than bypassing the adjustable filter.

`BANDWIDTH 11` through `23` use mode 0 and the existing capacitor curve.
`BANDWIDTH 24` through `48`, and `BANDWIDTH 0` (open), use mode 1 and a newly
measured curve. Crossing that boundary runs the complete PHY channel setup
and reapplies gain. Retuning preserves the selected mode. Captures still
save/restore the calibrated capacitor registers around the override.
`ALPF` remains a raw capacitor override in the currently selected PHY mode;
`LPF AUTO` restores that mode's PHY-selected digital filter.

## RF measurements

A user-provided HackRF One transmitted a fixed complex tone through antennas.
Only the C5 was retuned, keeping transmitter frequency, output and antenna
coupling constant. Captures used 80 MS/s IQ10, manual gain, CRC checks and
windowed FFTs. The HackRF RF amplifier and antenna power stayed disabled.

The final calibration compared modes outside busy Wi-Fi channels. At a fixed
2300 MHz RF tone, widest-setting relative response was approximately:

| Offset | Narrow mode | Wide mode |
| --- | --- | --- |
| ±12 MHz | -2.7 dB | -0.8 dB |
| ±20 MHz | -29 dB | -1.7 dB |
| ±24 MHz | near stopband floor | -3.1 dB |

The final comparison used normal firmware and public `BANDWIDTH 23`,
`BANDWIDTH 0` and `BANDWIDTH 40` commands, all with the same 2300 MHz tone.
No samples clipped. Full -3 dB width increased from approximately 24 MHz to
48 MHz; the 40 MHz request measured approximately -2.8 dB at ±20 MHz.
A fixed 5500 MHz tone also confirmed a wider path, with roughly
-0.4 dB at ±20 MHz and -3.2 dB at ±28 MHz in valid captures. A few isolated
5 GHz tone dropouts were excluded rather than treated as
filter notches. RF band, board calibration and conditions affect the curve;
the common 48 MHz maximum is conservative for the measured 5 GHz response.

Noise sweeps covered capacitor codes 0, 4, 8, 12, 16, 24, 32, 40, 48, 56, 60,
and 63 at 2300 and 5500 MHz, with 24 captures per setting. Tone checks at
codes 0/24/60 confirmed that capacitor control remains effective in mode 1.
Some 5 GHz noise snapshots clipped; those are not used as precision bandwidth
measurements. The MHz mapping is an approximate receive-path width, not an
isolated analog-filter specification or an alias-free guarantee at every rate.

Wide-mode capture-duration slopes remain approximately 80/40/20/10/8/4 MS/s.
Known RF tone offsets also agree with the nominal frequency axis. The wider
passband is not a sample-rate labelling change.

## UART root cause

All four UART Kconfig options excluded C5. `burst_serial_init()` therefore
did not install UART0, and UART `read_port()` compiled to `return 0`. Changing
host baud or console selection could not enable the missing transport.

C5 now enables UART0 by default at 2,000,000 baud, 8N1, TX GPIO11 and RX GPIO12.
These pins match the pinned ESP-IDF `soc/esp32c5/include/soc/uart_pins.h` and
[Espressif's pin documentation](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c5/schematic-checklist.html).
Native USB remains available. Old firmware must be rebuilt/reflashed.

On the attached board, `/dev/ttyACM0` is native Espressif USB Serial/JTAG;
`/dev/ttyACM1` is the QinHeng UART bridge. Opening the bridge resets this board
and re-enumerates native USB, invalidating an already-open native handle.
Opening the bridge first allowed bidirectional `ERR busy`/`RELEASE` checks.
Both cables were connected during testing; UART-only power/cold boot was not
checked.

## Validation

The host suite covers capacitor interpolation within each mode, the mode
boundary, malformed/out-of-range requests, and preservation of mode and exact
PLL frequency through real C5 receive preparation. All 24 host tests pass.

The final firmware checks cover both transports and both PHY modes at
2412/5500 MHz, all six rates, IQ8/IQ10 and 257/16380-sample captures. Further
checks exercise every advertised bandwidth and repeated boundary transitions
with manual gain and hardware AGC in both RF bands: 372 CRC-checked captures
passed before the final RF sweep. Temporary probe commands
are excluded from the normal firmware.

The tested firmware is bundled in the sibling `esp-web-sdr/firmware/esp32c5`
directory. Temporary builds, measurement files and duplicate firmware exports
were removed after verification. The original 2 MiB flash backup is at
`/tmp/esp32c5-original-flash-20260929.bin`.

The normal firmware with both fixes is flashed on the board. The HackRF
transmitter is stopped; the receiver was left at 2412 MHz, hardware gain,
open bandwidth and automatic digital filtering, with its serial lease released.
