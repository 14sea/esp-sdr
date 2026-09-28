#include "wifi_rx.h"

#include <stddef.h>
#include <string.h>

#include "esp_check.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "soc/interrupt_core0_reg.h"
#include "soc/interrupt_core1_reg.h"
#include "soc/interrupts.h"
#include "soc/soc.h"

#include "app_config.h"

#define HZ_PER_MHZ 1000000u
#define WIFI_REPORT_MIN_INTERVAL_US 5000u

static portMUX_TYPE s_config_mux = portMUX_INITIALIZER_UNLOCKED;
static wifi_rx_report_cb_t s_report_cb;
static wifi_rx_config_t s_config;
static volatile bool s_stream_armed;
static volatile uint32_t s_report_sequence = 1;
static volatile uint32_t s_dropped_reports;
static volatile uint32_t s_last_report_us;
static bool s_wifi_started;
static bool s_promiscuous_started;
static bool s_promiscuous_reports_enabled;

static uint32_t clamp_u32(uint32_t value, uint32_t min, uint32_t max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static uint8_t wifi_channel_from_freq_hz(uint32_t freq_hz)
{
    uint32_t freq_mhz = (freq_hz + (HZ_PER_MHZ / 2u)) / HZ_PER_MHZ;
    if (freq_mhz >= 2412u && freq_mhz <= 2472u) {
        return (uint8_t)clamp_u32((freq_mhz + 2u - 2407u) / 5u, 1u, 13u);
    }
    if (freq_mhz >= 2482u) {
        return 14u;
    }
    return 1u;
}

static wifi_second_chan_t wifi_second_channel_from_config(const wifi_rx_config_t *config)
{
    if (config->bw_mhz < 40u) {
        return WIFI_SECOND_CHAN_NONE;
    }
    if (config->second_chan == SECOND_CHAN_BELOW) {
        return WIFI_SECOND_CHAN_BELOW;
    }
    return WIFI_SECOND_CHAN_ABOVE;
}

static void update_wifi_channel(const wifi_rx_config_t *config)
{
    if (s_wifi_started) {
        wifi_second_chan_t second = wifi_second_channel_from_config(config);
        (void)esp_wifi_set_bandwidth(WIFI_IF_STA,
                                     second == WIFI_SECOND_CHAN_NONE ? WIFI_BW20 : WIFI_BW40);
        (void)esp_wifi_set_channel(wifi_channel_from_freq_hz(config->rf_freq_hz), second);
    }
}

static void copy_mac_or_zero(uint8_t dst[6], const uint8_t *payload, uint32_t payload_len,
                             uint32_t offset)
{
    if (payload_len >= offset + 6u) {
        memcpy(dst, payload + offset, 6u);
    } else {
        memset(dst, 0, 6u);
    }
}

static uint32_t wifi_payload_len(uint32_t frame_len, uint32_t frame_type,
                                 uint32_t frame_subtype, uint32_t frame_control)
{
    uint32_t header_len = 0u;
    if (frame_type == 0u) {
        header_len = 24u;
    } else if (frame_type == 1u) {
        header_len = (frame_subtype == 12u || frame_subtype == 13u) ? 10u : 16u;
    } else if (frame_type == 2u) {
        bool to_ds = (frame_control & (1u << 8)) != 0u;
        bool from_ds = (frame_control & (1u << 9)) != 0u;
        header_len = 24u + (to_ds && from_ds ? 6u : 0u) +
                     ((frame_subtype & 0x08u) != 0u ? 2u : 0u);
    }
    return frame_len > header_len + 4u ? frame_len - header_len - 4u : 0u;
}

static void wifi_promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (!s_stream_armed || s_config.stream_packets == 0u ||
        type == WIFI_PKT_MISC || buf == NULL || s_report_cb == NULL) {
        return;
    }
    const wifi_promiscuous_pkt_t *packet = (const wifi_promiscuous_pkt_t *)buf;
    uint32_t timestamp_us = packet->rx_ctrl.timestamp;
    uint32_t last_report_us = s_last_report_us;
    if ((uint32_t)(timestamp_us - last_report_us) < WIFI_REPORT_MIN_INTERVAL_US) {
        ++s_dropped_reports;
        return;
    }
    s_last_report_us = timestamp_us;

    const uint8_t *payload = packet->payload;
    uint32_t frame_len = packet->rx_ctrl.sig_len;
    if (frame_len < 2u) {
        return;
    }

    uint32_t frame_control = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8);
    uint32_t frame_type = (frame_control >> 2) & 0x3u;
    uint32_t frame_subtype = (frame_control >> 4) & 0xfu;

    wifi_packet_report_t report = {
        .magic = {'W', 'P', 'K', '1'},
        .sequence = s_report_sequence++,
        .rx_timestamp_us = timestamp_us,
        .channel = packet->rx_ctrl.channel,
        .rssi = packet->rx_ctrl.rssi,
        .rate = packet->rx_ctrl.rate,
        .sig_mode = packet->rx_ctrl.cur_bb_format,
        .mcs = 0u,
        .frame_type = frame_type,
        .frame_subtype = frame_subtype,
        .frame_control = frame_control,
        .frame_len = frame_len,
        .payload_len = wifi_payload_len(frame_len, frame_type, frame_subtype, frame_control),
        .dropped_reports = s_dropped_reports,
    };
    copy_mac_or_zero(report.receiver_mac, payload, frame_len, 4u);
    copy_mac_or_zero(report.sender_mac, payload, frame_len, 10u);
    copy_mac_or_zero(report.bssid, payload, frame_len, 16u);
    report.crc32 = esp_rom_crc32_le(0u, (const uint8_t *)&report,
                                    offsetof(wifi_packet_report_t, crc32));
    if (!s_report_cb(&report)) {
        ++s_dropped_reports;
    }
}

static void update_promiscuous_state(void)
{
    bool want_reports = s_config.stream_packets != 0u;
    bool want_promiscuous =
#if CONFIG_IDF_TARGET_ESP32S31
        true;
#else
        want_reports;
#endif
    if (!want_promiscuous && !s_promiscuous_started) {
        return;
    }
    if (want_promiscuous == s_promiscuous_started &&
        want_reports == s_promiscuous_reports_enabled) {
        return;
    }
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT |
                       WIFI_PROMIS_FILTER_MASK_DATA,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    if (want_reports) {
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_rx_cb));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(NULL));
    }
    s_promiscuous_reports_enabled = want_reports;
    if (want_promiscuous && !s_promiscuous_started) {
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
        s_promiscuous_started = true;
    } else if (!want_promiscuous && s_promiscuous_started) {
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
        s_promiscuous_started = false;
    }
}

void wifi_rx_init(wifi_rx_report_cb_t report_cb)
{
    s_report_cb = report_cb;
    if (!s_wifi_started) {
        wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
        wifi_config.static_rx_buf_num = 2;
        wifi_config.dynamic_rx_buf_num = 2;
        wifi_config.dynamic_tx_buf_num = 1;
        wifi_config.rx_mgmt_buf_num = 1;
        wifi_config.ampdu_rx_enable = 0;
        wifi_config.ampdu_tx_enable = 0;
        wifi_config.rx_ba_win = 0;
        wifi_config.mgmt_sbuf_num = 6;
        wifi_config.nvs_enable = 0;
        ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        s_wifi_started = true;
        update_wifi_channel(&s_config);
    }
    update_promiscuous_state();

}

void wifi_rx_apply_config(const wifi_rx_config_t *config)
{
    taskENTER_CRITICAL(&s_config_mux);
    s_config = *config;
    taskEXIT_CRITICAL(&s_config_mux);
    if (config->stream_packets != 0u) {
        update_wifi_channel(config);
    }
    if (s_wifi_started) {
        update_promiscuous_state();
    }
}

void wifi_rx_set_stream_armed(bool armed)
{
    s_stream_armed = armed;
}
