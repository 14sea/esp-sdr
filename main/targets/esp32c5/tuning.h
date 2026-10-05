#pragma once
extern void phy_set_chanfreq(unsigned mhz, unsigned mode);
extern void phy_set_rf_freq_offset(unsigned crystal, unsigned mhz, int offset);
extern unsigned char phy_param[];
static void c5_set_chan(unsigned mhz, unsigned mode) {
    /* The pinned PHY selects its 5 GHz path above 3000 MHz. Initialize band
     * calibration before programming exact PLL MHz.
     * phy_set_freq goes back through channel conversion on C5; bypass it.
     * phy_param[49] is the crystal selector used by phy_chip_set_chan.
     *
     * The RX DC-offset correction follows the calibration frequency. With
     * 5180 MHz for every 5 GHz request, the offset grows with distance from
     * 5180 MHz until, at high gain, it reaches the ADC rail and the hardware
     * AGC cycles between gain steps instead of settling. Above 5180 MHz,
     * calibrate at the requested frequency instead. Requests from 3000 to
     * 5180 MHz keep 5180 MHz: down to about 4200 MHz that gives the same
     * result, and below it the PHY responds differently (other DC offset and
     * noise floor), which is left unchanged here.
     *
     * Between 5320 and 5410 MHz, the gap between the Wi-Fi sub-bands, the
     * PHY answers a request with its upper-band values, which there leave
     * a larger offset than 5320 MHz does and can start the cycle again.
     * Use 5320 MHz for that range. */
    unsigned calibration = mhz > 5320 && mhz < 5410 ? 5320
                         : mhz > 5180 ? mhz : mhz > 3000 ? 5180 : 2412;
    phy_set_chanfreq(calibration, mode);
    phy_set_rf_freq_offset(phy_param[49], mhz, 0);
}
#define phy_chip_set_chan c5_set_chan
