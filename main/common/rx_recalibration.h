#pragma once

/* Receiver and capture must be stopped. Measures fresh RX DC and loopback IQ
 * coefficients at mhz, then rebuilds the gain tables. Caller restores its
 * channel, filter, RX power and gain settings afterwards. */
void rx_recalibrate(unsigned mhz);

#include <stdbool.h>
/* True once the PHY's temperature tracking has recalibrated behind the
 * receiver's back since the last rx_recalibrate(); only the C5 reports it. */
#if CONFIG_IDF_TARGET_ESP32C5
bool rx_recalibration_stale(void);
#else
static inline bool rx_recalibration_stale(void) { return false; }
#endif
