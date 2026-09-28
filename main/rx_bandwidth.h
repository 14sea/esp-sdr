#pragma once
#include <stdint.h>
/* C61: sensor-firmware/main/iq/modem.c noise-floor calibration, CBW20.
 * Approximate two-sided usable bandwidth, not a precision -3 dB specification.
 * S31 retains its separately characterized 21 MHz anchor.
 * S3 curve: measured noise spectrum at 2300 MHz, gain 75, 80 MS/s;
 * 12 snapshots/code, median windowed FFTs, approximate -3 dB full width. */
#if CONFIG_IDF_TARGET_ESP32C5
#define RX_BANDWIDTH_MIN 11u
#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C6
#define RX_BANDWIDTH_MIN 12u
#else
#define RX_BANDWIDTH_MIN 13u
#endif
#if CONFIG_IDF_TARGET_ESP32
#define RX_BANDWIDTH_MAX 67u
#elif CONFIG_IDF_TARGET_ESP32C5
#define RX_BANDWIDTH_MAX 23u
#elif CONFIG_IDF_TARGET_ESP32S3
#define RX_BANDWIDTH_MAX 69u
#else
#define RX_BANDWIDTH_MAX 54u
#endif
static inline uint8_t rx_bandwidth_dcap(unsigned mhz) {
    static const struct { uint8_t dcap,mhz; } cal[]={
#if CONFIG_IDF_TARGET_ESP32
        /* Original ESP32: seven-bit BBTOP 1/2, 2472 MHz, gain 72, IQ10.
         * Code 0 is wider than the measured span. Numeric max uses code 8. */
        {8,67},{12,55},{16,48},{24,38},{32,32},{48,25},
        {64,20},{80,17},{96,15},{112,14},{127,12}
#elif CONFIG_IDF_TARGET_ESP32C5
        /* C5: BBTOP 6/7; median noise FFTs at 2300/5500 MHz,
         * 80 MS/s IQ10, default digital filtering. Approximate full width. */
        {0,23},{4,22},{8,21},{12,20},{16,18},{24,16},
        {32,15},{40,13},{48,12},{60,11}
#elif CONFIG_IDF_TARGET_ESP32C6
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
    if(!mhz)return 0;
    if(mhz>=RX_BANDWIDTH_MAX)return cal[0].dcap;
    if(mhz<=RX_BANDWIDTH_MIN)return cal[sizeof(cal)/sizeof(cal[0])-1].dcap;
    for(unsigned i=1;i<sizeof(cal)/sizeof(cal[0]);i++)if(mhz>=cal[i].mhz) {
        unsigned span=cal[i-1].mhz-cal[i].mhz;
        return cal[i-1].dcap+((cal[i].dcap-cal[i-1].dcap)*(cal[i-1].mhz-mhz)+span/2)/span;
    }
    return cal[sizeof(cal)/sizeof(cal[0])-1].dcap;
}
