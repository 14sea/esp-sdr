#pragma once
#include <stdint.h>
/* C61: sensor-firmware/main/iq/modem.c noise-floor calibration, CBW20.
 * Approximate two-sided usable bandwidth, not a precision -3 dB specification.
 * S31 retains its separately characterized 21 MHz anchor.
 * S3 curve: measured noise spectrum at 2300 MHz, gain 75, 80 MS/s;
 * 12 snapshots/code, median windowed FFTs, approximate -3 dB full width. */
#if CONFIG_IDF_TARGET_ESP32C6
#define RX_BANDWIDTH_MIN 12u
#else
#define RX_BANDWIDTH_MIN 13u
#endif
#if CONFIG_IDF_TARGET_ESP32S3
#define RX_BANDWIDTH_MAX 69u
#else
#define RX_BANDWIDTH_MAX 54u
#endif
static inline uint8_t rx_bandwidth_dcap(unsigned mhz) {
    static const struct { uint8_t dcap,mhz; } cal[]={
#if CONFIG_IDF_TARGET_ESP32C6
        /* C6: median noise FFTs at 2484 MHz, gain 79, 80 MS/s. */
        {0,54},{4,48},{8,39},{12,33},{16,28},{24,23},
        {32,20},{40,17},{48,15},{60,12}
#elif CONFIG_IDF_TARGET_ESP32S3
        {0,69},{4,51},{8,45},{16,33},{24,25},{32,21},{48,16},{60,13}
#else
        {0,54},{8,36},{16,30},
#if CONFIG_IDF_TARGET_ESP32S31
        {28,21},
#else
        {24,21},
#endif
        {32,18},{48,15},{60,13}
#endif
    };
    if(!mhz || mhz>=RX_BANDWIDTH_MAX)return 0;
    if(mhz<=RX_BANDWIDTH_MIN)return 60;
    for(unsigned i=1;i<sizeof(cal)/sizeof(cal[0]);i++)if(mhz>=cal[i].mhz) {
        unsigned span=cal[i-1].mhz-cal[i].mhz;
        return cal[i-1].dcap+((cal[i].dcap-cal[i-1].dcap)*(cal[i-1].mhz-mhz)+span/2)/span;
    }
    return 60;
}
