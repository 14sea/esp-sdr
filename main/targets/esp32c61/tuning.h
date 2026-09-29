/* Extended RX tuning bypasses Wi-Fi channel conversion/calibration indices. */
#pragma once
#include "rx_tuning.h"
extern void phy_chip_set_chan(unsigned mhz, unsigned mode);
extern void phy_set_freq(unsigned mhz, int offset_khz);
static bool frequency_valid(unsigned mhz) {
    return rx_frequency_valid(mhz);
}
static void c61_set_chan(unsigned mhz, unsigned mode) {
    bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz-2412)%5 == 0) || mhz == 2484;
    phy_chip_set_chan(channel ? mhz : 2412, mode);
    if (!channel) phy_set_freq(mhz, 0);
}
#define phy_chip_set_chan c61_set_chan
