/* ESP32-S2 burst SDR over native USB-OTG CDC and optional UART0. */
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_phy_cert_test.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_memory_layout.h"
#include "nvs_flash.h"
#include "soc/soc.h"

#include "burst_serial.h"
#include "burst_gpio.h"
#include "burst_version.h"
#include "spectrum.h"
#include "rx_recalibration.h"
#include "rx_tuning.h"
#include "rx_lo.h"
#include "esp_rom_sys.h"
#include "pll.h"

/* S2 vendor adctrig uses SRAM at 0x3fff0000 with owner mask 7.
 * Reserve the first three 16 KiB banks and both CPU aliases. Leave the top
 * bank accessible for ROM USB state and startup/task memory. */
SOC_RESERVE_MEMORY_REGION(0x3fff0000, 0x3fffc000, s2_rf_dump);
#define IQ_WORDS       12284u
#define IQ_BUFFER      ((uint32_t *)0x3fff0000)
#define SRAM_OWNER_REG 0x3f4c10dcu
extern void adctrig(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
extern void rom_stop_tx_tone(unsigned);
#define phy_rom_stop_tx_tone rom_stop_tx_tone
extern void rom_pbus_workmode(void);
#define phy_pbus_workmode rom_pbus_workmode
extern void rom_pbus_xpd_rx_on(unsigned);
#define phy_pbus_xpd_rx_on rom_pbus_xpd_rx_on
extern void rom_pbus_xpd_tx_off(void);
#define phy_pbus_xpd_tx_off rom_pbus_xpd_tx_off
extern void rom_set_rxclk_en(unsigned);
#define phy_set_rxclk_en rom_set_rxclk_en
extern void set_chanfreq(unsigned, unsigned);
extern void rom_set_rf_freq_offset(unsigned, unsigned, int);
static void s2_tune(unsigned mhz) {
    static unsigned calibrated_mhz;
    if (calibrated_mhz != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    rx_lo_plan_t plan = rx_lo_plan(mhz);
    rx_lo_select(false);
    bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484;
    set_chanfreq(channel ? mhz : 2412, 0);
    if (!channel) {
        rom_set_rf_freq_offset(0, plan.mhz, plan.offset_khz);
        (void)s2_pll_calibrate();
    }
}

#define S2_FREQ_MIN RX_FREQ_MIN
#define S2_FREQ_MAX RX_FREQ_MAX
static bool frequency_valid(unsigned mhz) {
    return mhz >= S2_FREQ_MIN && mhz <= S2_FREQ_MAX;
}
static unsigned frequency_mhz = 2412;
static bool rx_ready;
#ifdef S2_RF_PROBE
static unsigned rx_clock = 0;
static unsigned rx_source, rx_mode, rx_flag, rx_wide, rx_prep = 3, rx_pack, rx_agc;
#else
enum { rx_source = 0, rx_mode = 0, rx_flag = 0, rx_wide = 0, rx_prep = 3, rx_pack = 0, rx_agc = 0 };
#endif
static void force_rx_gain(unsigned manual, unsigned index, unsigned unused) {
    (void)unused;
    uint32_t value = REG_READ(0x6001c02c);
    value = (value & ~0xff800000u) | (manual ? ((index << 24) | (1u << 23)) : 0);
    REG_WRITE(0x6001c02c, value);
}
static int rx_filter = -1; /* -1 restores the PHY-calibrated automatic mode. */
extern unsigned rom_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void rom_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
/* Apply only around an RX snapshot; restore before any retune. */
static unsigned rx_filter_saved[2];
static void rx_filter_apply(void) {
    for (unsigned j = 0; j < 2; j++) {
        rx_filter_saved[j] = rom_chip_i2c_readReg(0x67, 1, 4 + j);
        if (rx_filter >= 0)
            rom_chip_i2c_writeReg(0x67, 1, 4 + j, (rx_filter_saved[j] & ~63u) | (unsigned)rx_filter);
    }
}
static void rx_filter_restore(void) {
    if (rx_filter >= 0)
        for (unsigned j = 0; j < 2; j++) rom_chip_i2c_writeReg(0x67, 1, 4 + j, rx_filter_saved[j]);
}
/* Required by the stock RF test archive; no shell is exposed. */
int cmd_parse(char *cmd, char *name, int *argc, char **argv) {
    (void)cmd;
    (void)name;
    (void)argc;
    (void)argv;
    return -1;
}
#define send_bytes burst_serial_send
static void reply(const char *s) {
    (void)send_bytes(s, strlen(s));
}
#include "burst_gain.h"
#include "burst_limits.h"

static void prepare_rx(void) {
    if (rx_ready && !rx_recalibration_stale()) return;
    if (rx_prep == 1) {
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        force_rx_gain(1, 55, 0);
        rx_ready = true;
        return;
    }
    if (rx_prep == 2) {
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        rx_ready = true;
        return;
    }
    s2_tune(frequency_mhz);
    phy_rom_stop_tx_tone(1);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
    gain_apply();
    rx_lo_select(rx_lo_plan(frequency_mhz).alternate);
    esp_rom_delay_us(3000);
    rx_ready = true;
}
#include "burst_capture.h"
static bool acquire_iq(unsigned n, unsigned divider, unsigned *capture_us) {
    if (divider != 0 && divider != 1 && divider != 6) {
        reply("ERR rate\n");
        return false;
    }
    prepare_rx();

    for (unsigned j = 0; j < n + 4; j++) IQ_BUFFER[j] = 0xa5a0055au;
    rx_filter_apply();
    uint32_t owner = REG_READ(SRAM_OWNER_REG);
    int64_t start = esp_timer_get_time();
    REG_WRITE(0x60033d64, 0);
    REG_WRITE(0x60033d98, rx_pack | ((rx_pack + 1) << 6) | ((rx_pack + 2) << 12) | ((rx_pack + 3) << 18) |
                              (rx_agc << 24));
    REG_WRITE(SRAM_OWNER_REG, (owner & ~15u) | 7u);
    /* Timing slopes measured on S2: ~12.5/25/62.5 ns per sample.
     * Absolute RF/sample-rate calibration remains outstanding. */
    uint32_t rate_bits = divider == 1 ? (1u << 15) : divider == 6 ? (1u << 16) : 0;
    uint32_t ctrl =
        0x80000000u | rate_bits | (rx_wide << 17) | (rx_flag << 16) | (rx_source << 20) | (rx_mode << 28) | n;
#ifdef S2_RF_PROBE
    ctrl |= rx_clock << 15;
#endif
    REG_WRITE(0x60033d64, ctrl);
    REG_WRITE(0x60033d64, ctrl | (1u << 19));
    REG_WRITE(0x60033d64, ctrl);
    while (!(REG_READ(0x60033d64) & (1u << 18)) && esp_timer_get_time() - start < 20000) {}
    bool done = (REG_READ(0x60033d64) & (1u << 18)) != 0;
    uint32_t elapsed = esp_timer_get_time() - start;
    REG_WRITE(0x60033d64, 0);
    REG_WRITE(SRAM_OWNER_REG, owner);
    rx_filter_restore();
    if (!done) {
        reply("ERR capture_timeout\n");
        return false;
    }
    for (unsigned j = n; j < n + 4; j++)
        if (IQ_BUFFER[j] != 0xa5a0055au) {
            reply("ERR capture_overrun\n");
            return false;
        }
    for (unsigned j = 0; j < n; j++) {
        if (IQ_BUFFER[j] == 0xa5a0055au) {
            reply("ERR capture_timeout\n");
            return false;
        }
    }
    *capture_us = elapsed;
    return true;
}

static bool spectrum_acquire(unsigned n, unsigned rate, const uint32_t **data, unsigned *elapsed) {
    bool ok = acquire_iq(n, rate, elapsed);
    *data = IQ_BUFFER;
    return ok;
}

#include "ring_probe.h"
#include "dump_probe.h"

static bool capture(unsigned n, unsigned divider, unsigned format) {
    unsigned elapsed;
    if (!acquire_iq(n, divider, &elapsed)) return false;
    return burst_send_capture(n, format, elapsed);
}

#define BURST_ID       "S2SDR"
#define BURST_MAX_RATE 6
#include "burst_commands.h"

static void handle_command(char *line) {
    if (burst_version_command(line)) return;
    if (burst_gpio_command(line)) return;
#ifdef RING_PROBE
    if (ring_probe_command(line)) return;
#endif
#ifdef S2_RF_PROBE
    if (dump_probe_command(line)) return;
#endif
    if (spectrum_command(line, frequency_mhz, spectrum_acquire)) return;
    if (limits_command(line)) return;
    if (gain_command(line)) return;
    if (burst_common_command(line)) return;
    unsigned n;
    char extra;
    if (!strcmp(line, "CAPS")) {
        reply("CAPS VERSION GPIO SPEC SPECN SPECCAPS SPECSTAT DCT UARTBAUD RXLIMITS SERIALLEASE "
#if CONFIG_ESP_SDR_UART_ENABLED
              "DUALSERIAL "
#endif
              "TUNEEXT RX40 RX16 LPFANA GAIN HWAGC IQ8\n");
    } else if (sscanf(line, "BANDWIDTH %u %c", &n, &extra) == 1 &&
               (!n || (n >= RX_BANDWIDTH_MIN && n <= RX_BANDWIDTH_MAX))) {
        rx_filter = rx_bandwidth_dcap(n);
        reply("OK\n");
    } else if (!strcmp(line, "LPF AUTO")) {
        rx_filter = -1;
        reply("OK\n");
    } else if (sscanf(line, "LPF %u %c", &n, &extra) == 1 && n <= 63) {
        rx_filter = n;
        reply("OK\n");
    } else if (!strcmp(line, "LPF?")) {
        char answer[80];
        snprintf(answer, sizeof(answer), "LPF %d %u %u\n", rx_filter, rom_chip_i2c_readReg(0x67, 1, 4) & 63,
                 rom_chip_i2c_readReg(0x67, 1, 5) & 63);
        reply(answer);
    } else
        reply("ERR command\n");
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    prepare_rx();
    esp_log_level_set("*", ESP_LOG_NONE);
    /* USB may be unplugged when the host uses the UART bridge. */

    burst_serial_init();
    burst_serial_serve(handle_command);
}
