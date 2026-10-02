#pragma once
#include <stdbool.h>

/* Qualified on ESP32, S2, S3 and C3: CKGEN plan B divides the ordinary receive LO by
 * 6/5. Keep the vendor calibration in normal mode and select the final mode
 * only after RX setup. The frequency command always names the receive LO.
 * 1842..2209 MHz maps into the conservative 2210..2651 MHz PLL envelope;
 * other requests retain the existing direct-PLL tuning attempt. */
typedef struct {
    unsigned mhz;
    int offset_khz;
    bool alternate;
} rx_lo_plan_t;

static inline rx_lo_plan_t rx_lo_plan(unsigned mhz) {
    bool alternate = mhz >= 1842 && mhz < 2210;
    unsigned khz = mhz * (alternate ? 1200u : 1000u);
    return (rx_lo_plan_t){khz / 1000u, (int)(khz % 1000u), alternate};
}

/* These aliases are deliberately limited to radios whose selector and LO
 * ratio have been checked. Newer chips do not share this register layout. */
#if CONFIG_IDF_TARGET_ESP32
extern unsigned ram_chip_i2c_readReg(unsigned,unsigned,unsigned);
extern void ram_chip_i2c_writeReg(unsigned,unsigned,unsigned,unsigned);
#define rx_lo_read ram_chip_i2c_readReg
#define rx_lo_write ram_chip_i2c_writeReg
#define RX_LO_HOST 4
#elif CONFIG_IDF_TARGET_ESP32C3
extern unsigned rom1_chip_i2c_readReg(unsigned,unsigned,unsigned);
extern void rom1_chip_i2c_writeReg(unsigned,unsigned,unsigned,unsigned);
#define rx_lo_read rom1_chip_i2c_readReg
#define rx_lo_write rom1_chip_i2c_writeReg
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
extern unsigned rom_chip_i2c_readReg(unsigned,unsigned,unsigned);
extern void rom_chip_i2c_writeReg(unsigned,unsigned,unsigned,unsigned);
#define rx_lo_read rom_chip_i2c_readReg
#define rx_lo_write rom_chip_i2c_writeReg
#else
#error Unqualified alternate LO selector
#endif

#ifndef RX_LO_HOST
#define RX_LO_HOST 1
#endif

static inline bool rx_lo_select(bool alternate) {
    unsigned old = rx_lo_read(0x65, RX_LO_HOST, 0);
    unsigned value = (old & ~0x10u) | (alternate ? 0x10u : 0u);
    if (value != old) rx_lo_write(0x65, RX_LO_HOST, 0, value);
    return value != old;
}
