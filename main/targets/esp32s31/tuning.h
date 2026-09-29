#pragma once
/* Keep out-of-band requests out of the channel calibration/indexing path. */
static void s31_tune(unsigned mhz) {
    bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz-2412)%5 == 0) || mhz == 2484;
    phy_chip_set_chan(channel ? mhz : 2412, 0);
    phy_set_freq(mhz, 0);
}
