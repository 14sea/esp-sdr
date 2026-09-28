#pragma once

#include <stdbool.h>
#include <stdint.h>

#define WIFI_RX_REPORT_MAGIC "WPK1"

typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t sequence;
    uint32_t rx_timestamp_us;
    uint32_t channel;
    int32_t rssi;
    uint32_t rate;
    uint32_t sig_mode;
    uint32_t mcs;
    uint32_t frame_type;
    uint32_t frame_subtype;
    uint32_t frame_control;
    uint32_t frame_len;
    uint32_t payload_len;
    uint8_t receiver_mac[6];
    uint8_t sender_mac[6];
    uint8_t bssid[6];
    uint8_t reserved[2];
    uint32_t dropped_reports;
    uint32_t crc32;
} wifi_packet_report_t;

typedef struct {
    uint32_t rf_freq_hz;
    uint32_t bw_mhz;
    uint32_t second_chan;
    uint32_t stream_packets;
} wifi_rx_config_t;

typedef bool (*wifi_rx_report_cb_t)(const wifi_packet_report_t *report);

void wifi_rx_init(wifi_rx_report_cb_t report_cb);
void wifi_rx_apply_config(const wifi_rx_config_t *config);
void wifi_rx_set_stream_armed(bool armed);
