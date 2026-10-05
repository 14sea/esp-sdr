#pragma once
extern void phy_set_chanfreq(unsigned mhz, unsigned mode);
extern void phy_set_rf_freq_offset(unsigned crystal, unsigned mhz, int offset);
extern unsigned char phy_param[];
/* The RX DC-offset correction follows the calibration frequency. With
 * 5180 MHz for every 5 GHz request, the offset grows with distance from
 * 5180 MHz until, at high gain, it reaches the ADC rail and the hardware AGC
 * cycles between gain steps instead of settling. */
static unsigned c5_calibration_mhz(unsigned mhz) {
    /* 2.4 GHz path, unchanged. */
    if (mhz <= 3000) return 2412;
    /* Unchanged. Down to about 4200 MHz the requested frequency would give
     * the same result; below that the PHY responds differently (other DC
     * offset and noise floor), which is deliberately left alone. */
    if (mhz <= 5180) return 5180;
    /* Gap between the Wi-Fi sub-bands: the PHY answers a request here with
     * its upper-band values, which leave a larger offset than 5320 MHz does
     * and can start the cycle again. */
    if (mhz > 5320 && mhz < 5410) return 5320;
    return mhz;
}
static void c5_set_chan(unsigned mhz, unsigned mode) {
    /* The pinned PHY selects its 5 GHz path above 3000 MHz. Initialize band
     * calibration before programming exact PLL MHz.
     * phy_set_freq goes back through channel conversion on C5; bypass it.
     * phy_param[49] is the crystal selector used by phy_chip_set_chan. */
    phy_set_chanfreq(c5_calibration_mhz(mhz), mode);
    phy_set_rf_freq_offset(phy_param[49], mhz, 0);
}
#define phy_chip_set_chan c5_set_chan
