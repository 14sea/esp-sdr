#pragma once
extern void phy_set_chanfreq(unsigned mhz, unsigned mode);
extern void phy_set_rf_freq_offset(unsigned crystal, unsigned mhz, int offset);
extern unsigned char phy_param[];
/* The PHY's RX DC-offset correction follows the calibration channel. Using
 * 5180 MHz for every 5 GHz request leaves an offset that grows with distance
 * from 5180 MHz; at high gain it reaches the ADC rail and the hardware AGC
 * cycles between gain steps instead of settling. Calibrate on the nearest
 * 20 MHz Wi-Fi channel centre instead. */
static unsigned c5_calibration_mhz(unsigned mhz) {
    static const unsigned short centres[] = {
        5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320,
        5500, 5520, 5540, 5560, 5580, 5600, 5620, 5640, 5660, 5680, 5700, 5720,
        5745, 5765, 5785, 5805, 5825 };
    if (mhz <= 3000) return 2412;
    unsigned best = centres[0], distance = ~0u;
    for (unsigned i = 0; i < sizeof(centres) / sizeof(centres[0]); i++) {
        unsigned d = centres[i] > mhz ? centres[i] - mhz : mhz - centres[i];
        if (d < distance) { distance = d; best = centres[i]; }
    }
    return best;
}
static void c5_set_chan(unsigned mhz, unsigned mode) {
    /* The pinned PHY selects its 5 GHz path above 3000 MHz. Initialize band
     * calibration on a real channel before programming exact PLL MHz.
     * phy_set_freq goes back through channel conversion on C5; bypass it.
     * phy_param[49] is the crystal selector used by phy_chip_set_chan. */
    phy_set_chanfreq(c5_calibration_mhz(mhz), mode);
    phy_set_rf_freq_offset(phy_param[49], mhz, 0);
}
#define phy_chip_set_chan c5_set_chan
