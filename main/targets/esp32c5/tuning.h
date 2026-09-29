#pragma once
extern void phy_set_chanfreq(unsigned mhz, unsigned mode);
extern void phy_set_rf_freq_offset(unsigned crystal, unsigned mhz, int offset);
extern unsigned char phy_param[];
static void c5_set_chan(unsigned mhz, unsigned mode) {
    /* The pinned PHY selects its 5 GHz path above 3000 MHz. Initialize band
     * calibration on a real channel before programming exact PLL MHz.
     * phy_set_freq goes back through channel conversion on C5; bypass it.
     * phy_param[49] is the crystal selector used by phy_chip_set_chan. */
    phy_set_chanfreq(mhz > 3000 ? 5180 : 2412, mode);
    phy_set_rf_freq_offset(phy_param[49], mhz, 0);
}
#define phy_chip_set_chan c5_set_chan
