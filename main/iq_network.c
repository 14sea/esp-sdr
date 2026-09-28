#include "iq_network.h"
#include "gaintable.h"
#include "iq_usb.h"
#include "ringbuffer.h"

#include <inttypes.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_cache.h"
#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "esp_private/eth_mac_esp_dma.h"
#include "esp_random.h"
#include "esp_rom_crc.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/dma_types.h"
#include "hal/emac_hal.h"
#include "lwip/etharp.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"
#include "lwip/tcpip.h"
#include "mdns.h"

#define HTTP_BODY_MAX_BYTES 4096u
#define UDP_PACKET_BYTES                                                       \
  (sizeof(iq_udp_header_t) + IQ_UDP_FRAGMENT_PAYLOAD_BYTES)
#define ETH_HEADER_BYTES 14u
#define IPV4_HEADER_BYTES 20u
#define UDP_HEADER_BYTES 8u
#define TX_FRAME_BYTES                                                        \
  (ETH_HEADER_BYTES + IPV4_HEADER_BYTES + UDP_HEADER_BYTES + UDP_PACKET_BYTES)
#define STREAM_ARM_DELAY_US 100000u
#define IQ_ETH_TX_MUTEX_TIMEOUT_MS 250u
static const char *TAG = "iq_network";
static iq_network_callbacks_t s_callbacks;
static esp_eth_handle_t s_eth_handle;
static esp_netif_t *s_eth_netif;
static httpd_handle_t s_http_server;
static esp_timer_handle_t s_stream_arm_timer;
static portMUX_TYPE s_stream_mux = portMUX_INITIALIZER_UNLOCKED;
static struct sockaddr_in s_stream_destination;
static uint8_t s_stream_destination_mac[6];
static uint8_t s_source_mac[6];
static volatile bool s_stream_armed;
static volatile iq_stream_owner_t s_stream_owner;
static volatile uint32_t s_stream_format;

static void stream_arm_timer_callback(void *arg) {
  (void)arg;
  taskENTER_CRITICAL(&s_stream_mux);
  s_stream_armed = true;
  taskEXIT_CRITICAL(&s_stream_mux);
}

static volatile bool s_link_up;
static volatile bool s_has_ipv4;
static uint32_t s_ipv4_address;
static uint32_t s_stream_epoch;
static uint32_t s_datagram_sequence;
static volatile uint32_t s_udp_frames;
static volatile uint32_t s_udp_datagrams;
static volatile uint32_t s_udp_bytes;
static volatile uint32_t s_udp_send_errors;
static volatile uint32_t s_eth_tx_desc_completed;
static volatile uint32_t s_eth_tx_desc_errors;
static volatile uint32_t s_eth_tx_desc_underflows;
static volatile uint32_t s_eth_tx_desc_flushed;
static volatile uint32_t s_eth_tx_desc_carrier_errors;
static volatile uint32_t s_eth_dma_status;
/* The stream task is the sole writer. The GMAC driver copies a frame into its
 * DMA ring before returning, so this staging frame can be reused immediately. */
static uint8_t s_tx_frame[TX_FRAME_BYTES];
static uint8_t s_iq8_frame[IQ8_FRAME_WIRE_BYTES] __attribute__((aligned(4)));

/* Pinned-IDF driver prefix used only to hold the public transmit mutex across
 * all three UDP fragments of one IQ frame.  Calling the MAC once per fragment
 * while taking the mutex only once avoids six scheduler/semaphore operations
 * per 1024 IQ samples, while lwIP control traffic remains serialized. */
typedef struct {
  esp_eth_mediator_t mediator;
  esp_eth_phy_t *phy;
  esp_eth_mac_t *mac;
  esp_timer_handle_t check_link_timer;
  uint32_t check_link_period_ms;
  bool auto_nego_en;
  eth_speed_t speed;
  eth_duplex_t duplex;
  _Atomic eth_link_t link;
  atomic_int ref_count;
  void *priv;
  _Atomic int fsm;
  SemaphoreHandle_t transmit_mutex;
} iq_eth_driver_prefix_t;

typedef struct {
  emac_hal_context_t hal;
  uint32_t tx_desc_flags;
  uint32_t rx_desc_flags;
  void *descriptors;
  eth_dma_rx_descriptor_t *rx_desc;
  eth_dma_tx_descriptor_t *tx_desc;
  uint8_t *rx_buf[CONFIG_ETH_DMA_RX_BUFFER_NUM];
  uint8_t *tx_buf[CONFIG_ETH_DMA_TX_BUFFER_NUM];
} iq_emac_dma_prefix_t;

typedef struct {
  esp_eth_mac_t parent;
  esp_eth_mediator_t *eth;
  emac_hal_context_t hal;
  intr_handle_t intr_hdl;
  TaskHandle_t rx_task_hdl;
  iq_emac_dma_prefix_t *emac_dma_hndl;
} iq_emac_mac_prefix_t;

#define IQ_EMAC_TDES0_FS_FLAGS_MASK 0x0fcc0000u
#define IQ_EMAC_TDES0_LS_FLAGS_MASK 0x40000000u

static uint16_t ipv4_checksum(const uint8_t *header) {
  uint32_t sum = 0u;
  for (uint32_t i = 0u; i < IPV4_HEADER_BYTES; i += 2u)
    sum += ((uint16_t)header[i] << 8) | header[i + 1u];
  while (sum >> 16)
    sum = (sum & 0xffffu) + (sum >> 16);
  return (uint16_t)~sum;
}

static bool resolve_peer_mac(struct in_addr peer, uint8_t mac[6]) {
  ip4_addr_t peer_ip;
  ip4_addr_set_u32(&peer_ip, peer.s_addr);
  struct eth_addr *eth_addr = NULL;
  const ip4_addr_t *cached_ip = NULL;
  struct netif *netif = esp_netif_get_netif_impl(s_eth_netif);
  LOCK_TCPIP_CORE();
  int index = etharp_find_addr(netif, &peer_ip, &eth_addr, &cached_ip);
  if (index >= 0 && eth_addr != NULL)
    memcpy(mac, eth_addr->addr, 6u);
  UNLOCK_TCPIP_CORE();
  return index >= 0 && eth_addr != NULL;
}

static void build_udp_frame(const struct sockaddr_in *destination,
                            const uint8_t destination_mac[6],
                            size_t udp_payload_bytes,
                            uint16_t identification) {
  memcpy(s_tx_frame, destination_mac, 6u);
  memcpy(s_tx_frame + 6u, s_source_mac, 6u);
  s_tx_frame[12] = 0x08u;
  s_tx_frame[13] = 0x00u;

  uint8_t *ip = s_tx_frame + ETH_HEADER_BYTES;
  memset(ip, 0, IPV4_HEADER_BYTES + UDP_HEADER_BYTES);
  ip[0] = 0x45u;
  uint16_t ip_bytes =
      htons((uint16_t)(IPV4_HEADER_BYTES + UDP_HEADER_BYTES + udp_payload_bytes));
  memcpy(ip + 2u, &ip_bytes, sizeof(ip_bytes));
  uint16_t ip_id = htons(identification);
  memcpy(ip + 4u, &ip_id, sizeof(ip_id));
  ip[6] = 0x40u;
  ip[8] = 64u;
  ip[9] = IPPROTO_UDP;
  memcpy(ip + 12u, &s_ipv4_address, 4u);
  memcpy(ip + 16u, &destination->sin_addr.s_addr, 4u);
  uint16_t checksum = htons(ipv4_checksum(ip));
  memcpy(ip + 10u, &checksum, sizeof(checksum));

  uint8_t *udp = ip + IPV4_HEADER_BYTES;
  uint16_t source_port = htons(IQ_NETWORK_UDP_DEFAULT_PORT);
  uint16_t udp_bytes = htons((uint16_t)(UDP_HEADER_BYTES + udp_payload_bytes));
  memcpy(udp, &source_port, sizeof(source_port));
  memcpy(udp + 2u, &destination->sin_port, sizeof(destination->sin_port));
  memcpy(udp + 4u, &udp_bytes, sizeof(udp_bytes));
}

static esp_err_t yt8531_init(esp_eth_handle_t eth_handle) {
  bool autoneg = true;
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_S_AUTONEGO, &autoneg),
                      TAG, "enable YT8531 auto-negotiation");

  uint32_t value = 0xa001u;
  esp_eth_phy_reg_rw_data_t reg = {.reg_addr = 0x1eu, .reg_value_p = &value};
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &reg),
                      TAG, "select YT8531 chip config");
  reg.reg_addr = 0x1fu;
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_READ_PHY_REG, &reg),
                      TAG, "read YT8531 chip config");
  value |= BIT(8);
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &reg),
                      TAG, "set YT8531 RX delay");

  value = 0xa003u;
  reg.reg_addr = 0x1eu;
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &reg),
                      TAG, "select YT8531 RGMII config");
  reg.reg_addr = 0x1fu;
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_READ_PHY_REG, &reg),
                      TAG, "read YT8531 RGMII config");
  value = (value & ~0xffu) | (13u << 4) | 13u;
  return esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &reg);
}

static esp_err_t ethernet_driver_init(void) {
  ESP_LOGI(TAG, "creating S31 GMAC");
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  mac_config.flags |= ETH_MAC_FLAG_PIN_TO_CORE;
  /* The authenticated TX-IQ fast path is consumed directly by this task.
   * Keep it above lwIP and the stream service so the six DMA descriptors are
   * reclaimed promptly during a sustained host-to-radio transfer. */
  mac_config.rx_task_prio = 23u;
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = -1;
  phy_config.reset_gpio_num = 7;

  eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
  emac_config.smi_gpio.mdc_num = 5;
  emac_config.smi_gpio.mdio_num = 6;
  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
  ESP_RETURN_ON_FALSE(mac != NULL, ESP_ERR_NO_MEM, TAG, "create S31 GMAC");
  ESP_LOGI(TAG, "created S31 GMAC; creating PHY");
  esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
  if (phy == NULL) {
    mac->del(mac);
    return ESP_ERR_NO_MEM;
  }
  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
  ESP_LOGI(TAG, "created PHY; installing Ethernet driver");
  esp_err_t err = esp_eth_driver_install(&config, &s_eth_handle);
  if (err != ESP_OK) {
    mac->del(mac);
    phy->del(phy);
    return err;
  }
  ESP_RETURN_ON_ERROR(
      esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, s_source_mac), TAG,
      "read Ethernet MAC address");
  ESP_LOGI(TAG, "installed Ethernet driver; configuring YT8531");
  return yt8531_init(s_eth_handle);
}

static uint32_t json_u32(cJSON *object, const char *name, uint32_t value) {
  cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
  if (cJSON_IsNumber(item) && item->valuedouble >= 0.0 &&
      item->valuedouble <= UINT32_MAX) {
    return (uint32_t)item->valuedouble;
  }
  return value;
}

static int32_t json_i32(cJSON *object, const char *name, int32_t value) {
  cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
  if (cJSON_IsNumber(item) && item->valuedouble >= INT32_MIN &&
      item->valuedouble <= INT32_MAX) {
    return (int32_t)item->valuedouble;
  }
  return value;
}

static bool parse_config(cJSON *root, capture_config_t *c) {
  if (!cJSON_IsObject(root))
    return false;
  cJSON *o;
#define GET_OBJECT(name)                                                       \
  o = cJSON_GetObjectItemCaseSensitive(root, name);                            \
  if (cJSON_IsObject(o))
#define U32(name, field) c->field = json_u32(o, name, c->field)
  GET_OBJECT("stream") {
    U32("stream_wifi_packets", stream.stream_wifi_packets);
  }
  GET_OBJECT("radio") {
    U32("rf_freq_hz", radio.rf_freq_hz);
    c->radio.frequency_correction_ppb = json_i32(
        o, "frequency_correction_ppb", c->radio.frequency_correction_ppb);
  }
  GET_OBJECT("gain") {
    U32("gain_mode", gain.gain_mode);
    U32("rx_gain", gain.rx_gain);
    U32("expert_gain_word0", gain.expert_gain_word0);
    U32("expert_gain_word1", gain.expert_gain_word1);
    U32("expert_gain_word2", gain.expert_gain_word2);
  }
  GET_OBJECT("bandwidth") {
    U32("bw_mhz", bandwidth.bw_mhz);
    U32("second_chan", bandwidth.second_chan);
  }

  GET_OBJECT("iq_engine") {
    U32("adc_decimation", iq_engine.adc_decimation);
    U32("adc_source_sel", iq_engine.adc_source_sel);
  }
  GET_OBJECT("trigger") {
    U32("trigger_mode", trigger.trigger_mode);
    cJSON *a = cJSON_GetObjectItemCaseSensitive(o, "trigger_config");
    if (cJSON_IsArray(a)) {
      for (uint32_t i = 0; i < IQ_TRIGGER_CONFIG_WORDS; ++i) {
        cJSON *item = cJSON_GetArrayItem(a, i);
        if (cJSON_IsNumber(item) && item->valuedouble >= 0.0 &&
            item->valuedouble <= UINT32_MAX)
          c->trigger.trigger_config[i] = (uint32_t)item->valuedouble;
      }
    }
  }
  GET_OBJECT("rx_filter") {
    U32("filter_bw_mhz", rx_filter.filter_bw_mhz);
    U32("rx_filter_override", rx_filter.rx_filter_override);
    U32("rx_filter_mode", rx_filter.rx_filter_mode);
    U32("rx_filter_dcap", rx_filter.rx_filter_dcap);
  }

  GET_OBJECT("dc_offset") {
    U32("automatic", dc_offset.automatic);
  }
#undef U32
#undef GET_OBJECT
  return true;
}

static const char *validate_config(const capture_config_t *c) {
  const uint8_t gain_entry_count = gaintable_entry_count();
  if (c->gain.gain_mode > GAIN_MODE_EXPERT)
    return "gain_mode must be 0 (hardware AGC), 1 (manual), or 2 (expert)";
  if (c->gain.gain_mode == GAIN_MODE_MANUAL &&
      (gain_entry_count == 0u || c->gain.rx_gain >= gain_entry_count))
    return "manual rx_gain exceeds the calibrated gain table";
  if (c->radio.frequency_correction_ppb < -RF_CORRECTION_MAX_PPB ||
      c->radio.frequency_correction_ppb > RF_CORRECTION_MAX_PPB)
    return "frequency_correction_ppb must be in the range -100000..100000";
  if (c->dc_offset.automatic > 1u)
    return "dc_offset automatic must be 0 or 1";
  return NULL;
}

int iq_control_build_config_json(char *text, size_t cap) {
  capture_config_t config = {0};
  s_callbacks.get_config(&config);
  int n = snprintf(
      text, cap,
      "{\"stream\":{\"stream_wifi_packets\":%" PRIu32 "},"
      "\"radio\":{\"rf_freq_hz\":%" PRIu32
      ",\"frequency_correction_ppb\":%" PRId32 "},"
      "\"gain\":{\"gain_mode\":%" PRIu32 ",\"rx_gain\":%" PRIu32
      ",\"expert_gain_word0\":%" PRIu32 ",\"expert_gain_word1\":%" PRIu32
      ",\"expert_gain_word2\":%" PRIu32 "},"
      "\"bandwidth\":{\"bw_mhz\":%" PRIu32 ",\"second_chan\":%" PRIu32 "},"
      "\"iq_engine\":{\"adc_decimation\":%" PRIu32 ",\"adc_source_sel\":%" PRIu32 "},"
      "\"trigger\":{\"trigger_mode\":%" PRIu32 ",\"trigger_config\":[%" PRIu32
      ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
      ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
      ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]},"
      "\"rx_filter\":{\"filter_bw_mhz\":%" PRIu32
      ",\"rx_filter_override\":%" PRIu32 ",\"rx_filter_mode\":%" PRIu32
      ",\"rx_filter_dcap\":%" PRIu32 "},"
      "\"dc_offset\":{\"automatic\":%" PRIu32 "}}",
      config.stream.stream_wifi_packets, config.radio.rf_freq_hz,
      config.radio.frequency_correction_ppb,
      config.gain.gain_mode, config.gain.rx_gain,
      config.gain.expert_gain_word0,
      config.gain.expert_gain_word1, config.gain.expert_gain_word2,
      config.bandwidth.bw_mhz, config.bandwidth.second_chan,
      config.iq_engine.adc_decimation, config.iq_engine.adc_source_sel,
      config.trigger.trigger_mode, config.trigger.trigger_config[0],
      config.trigger.trigger_config[1], config.trigger.trigger_config[2],
      config.trigger.trigger_config[3], config.trigger.trigger_config[4],
      config.trigger.trigger_config[5], config.trigger.trigger_config[6],
      config.trigger.trigger_config[7], config.trigger.trigger_config[8],
      config.trigger.trigger_config[9], config.trigger.trigger_config[10],
      config.trigger.trigger_config[11], config.trigger.trigger_config[12],
      config.trigger.trigger_config[13], config.trigger.trigger_config[14],
      config.trigger.trigger_config[15], config.rx_filter.filter_bw_mhz,
      config.rx_filter.rx_filter_override, config.rx_filter.rx_filter_mode,
      config.rx_filter.rx_filter_dcap,
      config.dc_offset.automatic);
  return (n > 0 && (size_t)n < cap) ? n : -1;
}

static esp_err_t config_get_handler(httpd_req_t *req) {
  char text[1024];
  int n = iq_control_build_config_json(text, sizeof(text));
  ESP_RETURN_ON_FALSE(n > 0, ESP_ERR_INVALID_SIZE, TAG, "format config JSON");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, text, n);
}

static esp_err_t receive_json(httpd_req_t *req, cJSON **json) {
  if (req->content_len <= 0 || req->content_len >= HTTP_BODY_MAX_BYTES) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid JSON body size");
    return ESP_ERR_INVALID_SIZE;
  }
  char *body = malloc((size_t)req->content_len + 1u);
  if (body == NULL) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    return ESP_ERR_NO_MEM;
  }
  size_t received = 0;
  while (received < (size_t)req->content_len) {
    int n = httpd_req_recv(req, body + received, req->content_len - received);
    if (n <= 0) {
      free(body);
      return ESP_FAIL;
    }
    received += (size_t)n;
  }
  body[received] = '\0';
  *json = cJSON_ParseWithLength(body, received);
  free(body);
  if (*json == NULL) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "malformed JSON");
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

const char *iq_control_parse_config_json(struct cJSON *root,
                                         capture_config_t *config) {
  s_callbacks.get_config(config);
  if (!parse_config(root, config)) {
    return "configuration must be an object";
  }
  return validate_config(config);
}

static esp_err_t config_put_handler(httpd_req_t *req) {
  cJSON *json = NULL;
  ESP_RETURN_ON_ERROR(receive_json(req, &json), TAG, "receive config JSON");
  capture_config_t config = {0};
  const char *validation_error = iq_control_parse_config_json(json, &config);
  cJSON_Delete(json);
  if (validation_error != NULL) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, validation_error);
    return ESP_ERR_INVALID_ARG;
  }
  httpd_resp_set_status(req, "202 Accepted");
  esp_err_t response_err = httpd_resp_send(req, NULL, 0);
  /* Applying a radio configuration wakes the high-priority stream task and
   * can take several seconds.  Queue it only after the 202 response has been
   * handed to TCP, otherwise that task can preempt the HTTP server before the
   * client sees any response. */
  s_callbacks.apply_config(&config);
  return response_err;
}

static esp_err_t iq_eth_input(esp_eth_handle_t handle, uint8_t *buffer,
                              uint32_t length, void *priv, void *info) {
  (void)handle; (void)info;
  return esp_netif_receive((esp_netif_t *)priv, buffer, length, NULL);
}

int iq_control_build_status_json(char *text, size_t cap) {
  capture_config_t config = {0};
  s_callbacks.get_config(&config);
  uint32_t rx_decimation = config.iq_engine.adc_decimation;
  if (rx_decimation < 1u || rx_decimation > 10u) {
    rx_decimation = 1u;
  }
  const uint32_t rx_sample_rate_hz =
      IQ_RX_BASE_SAMPLE_RATE_HZ / rx_decimation;
  char ip[INET_ADDRSTRLEN] = "0.0.0.0";
  struct in_addr address = {.s_addr = s_ipv4_address};
  (void)inet_ntop(AF_INET, &address, ip, sizeof(ip));
  const uint8_t gain_entry_count = gaintable_entry_count();
  const unsigned gain_max = gain_entry_count > 0u
                                ? (unsigned)gain_entry_count - 1u
                                : RX_GAIN_MIN_DB;
  const iq_stream_owner_t owner = s_stream_owner;
  const char *owner_name = owner == IQ_STREAM_OWNER_ETH   ? "ethernet"
                           : owner == IQ_STREAM_OWNER_USB ? "usb"
                           : owner == IQ_STREAM_OWNER_SERIAL ? "serial"
                                                          : "none";
  const uint64_t hardware_time_ns =
      (uint64_t)esp_timer_get_time() * 1000u;
  int n = snprintf(
      text, cap,
      "{\"hostname\":\"" IQ_NETWORK_HOSTNAME ".local\",\"link_up\":%s,"
      "\"has_ipv4\":%s,\"ipv4\":\"%s\",\"streaming\":%s,"
      "\"rx_sample_rate_hz\":%" PRIu32
      ",\"rx_decimation\":%" PRIu32 ","
      "\"stream_owner\":\"%s\",\"usb_mounted\":%s,"
      "\"usb_frames\":%" PRIu32 ",\"usb_send_errors\":%" PRIu32
      ",\"usb_stream_format\":%" PRIu32
      ",\"stream_format\":%" PRIu32
      ","
      "\"config_applying\":%s,\"stream_epoch\":%" PRIu32
      ",\"reset_reason\":%u"
      ",\"hardware_time_ns\":%" PRIu64
      ",\"udp_frames\":%" PRIu32 ",\"udp_datagrams\":%" PRIu32
      ",\"udp_bytes\":%" PRIu32 ",\"udp_send_errors\":%" PRIu32
      ",\"firmware_dropped_chunks\":%" PRIu32
      ",\"source_chunk_index\":%" PRIu32 ",\"dcoc_diag\":%" PRIu32
      ",\"dc_offset_automatic\":%s,\"dcoc_active\":%s"
      ",\"adc_dump_cfg\":%" PRIu32 ",\"adc_dump_mode\":%" PRIu32 ","
      "\"manual_rx_gain\":{\"unit\":\"dB\",\"minimum\":%u,"
      "\"maximum\":%u,\"step\":%u}}",
      s_link_up ? "true" : "false", s_has_ipv4 ? "true" : "false", ip,
      s_stream_armed ? "true" : "false", rx_sample_rate_hz, rx_decimation,
      owner_name,
      iq_usb_mounted() ? "true" : "false", iq_usb_frames(),
      iq_usb_send_errors(), iq_usb_stream_format(), iq_network_stream_format(),
      s_callbacks.is_config_applying() ? "true" : "false", s_stream_epoch,
      (unsigned)esp_reset_reason(),
      hardware_time_ns,
      s_udp_frames, s_udp_datagrams, s_udp_bytes, s_udp_send_errors,
      s_callbacks.get_firmware_dropped_chunks(),
      s_callbacks.get_source_chunk_index(), s_callbacks.get_dcoc_diag(),
      config.dc_offset.automatic != 0u ? "true" : "false",
      s_callbacks.get_dcoc_active() ? "true" : "false",
      s_callbacks.get_adc_dump_cfg(), s_callbacks.get_adc_dump_mode(),
      RX_GAIN_MIN_DB, gain_max, RX_GAIN_STEP_DB);
  return (n > 0 && (size_t)n < cap) ? n : -1;
}

static esp_err_t send_status(httpd_req_t *req) {
  char text[2048];
  int n = iq_control_build_status_json(text, sizeof(text));
  ESP_RETURN_ON_FALSE(n > 0, ESP_ERR_INVALID_SIZE, TAG, "format status JSON");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, text, n);
}

static esp_err_t status_get_handler(httpd_req_t *req) {
  return send_status(req);
}

static esp_err_t stream_start_handler(httpd_req_t *req) {
  cJSON *json = NULL;
  ESP_RETURN_ON_ERROR(receive_json(req, &json), TAG, "receive stream JSON");
  cJSON *port_json = cJSON_GetObjectItemCaseSensitive(json, "port");
  if (!cJSON_IsNumber(port_json) || port_json->valuedouble < 1 ||
      port_json->valuedouble > 65535) {
    cJSON_Delete(json);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "port must be 1..65535");
    return ESP_ERR_INVALID_ARG;
  }
  uint16_t port = (uint16_t)port_json->valueint;
  uint32_t format = IQ_USB_FORMAT_FULL;
  cJSON *format_json =
      cJSON_GetObjectItemCaseSensitive(json, "stream_format");
  if (format_json != NULL) {
    if (!cJSON_IsNumber(format_json) || format_json->valuedouble < 0.0 ||
        format_json->valuedouble > (double)IQ_USB_FORMAT_INT8 ||
        format_json->valuedouble != (double)format_json->valueint) {
      cJSON_Delete(json);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                          "stream_format must be 0 or 1");
      return ESP_ERR_INVALID_ARG;
    }
    format = (uint32_t)format_json->valueint;
  }
  cJSON_Delete(json);

  struct sockaddr_storage peer_storage = {0};
  socklen_t peer_len = sizeof(peer_storage);
  int http_fd = httpd_req_to_sockfd(req);
  if (getpeername(http_fd, (struct sockaddr *)&peer_storage, &peer_len) != 0) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "cannot determine IPv4 peer");
    return ESP_FAIL;
  }
  struct sockaddr_in peer = {.sin_family = AF_INET};
  if (peer_storage.ss_family == AF_INET) {
    peer.sin_addr = ((struct sockaddr_in *)&peer_storage)->sin_addr;
  } else if (peer_storage.ss_family == AF_INET6) {
    const uint8_t *bytes =
        ((struct sockaddr_in6 *)&peer_storage)->sin6_addr.s6_addr;
    static const uint8_t mapped_prefix[12] = {0, 0, 0, 0, 0,    0,
                                              0, 0, 0, 0, 0xff, 0xff};
    if (memcmp(bytes, mapped_prefix, sizeof(mapped_prefix)) != 0) {
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                          "HTTP peer is not IPv4");
      return ESP_ERR_NOT_SUPPORTED;
    }
    memcpy(&peer.sin_addr.s_addr, bytes + 12, sizeof(peer.sin_addr.s_addr));
  } else {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "unknown HTTP peer family");
    return ESP_ERR_NOT_SUPPORTED;
  }
  peer.sin_port = htons(port);
  uint8_t destination_mac[6];
  if (!resolve_peer_mac(peer.sin_addr, destination_mac)) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "HTTP peer is not present in ARP cache");
    return ESP_FAIL;
  }
  taskENTER_CRITICAL(&s_stream_mux);
  s_stream_destination = peer;
  memcpy(s_stream_destination_mac, destination_mac,
         sizeof(s_stream_destination_mac));
  taskEXIT_CRITICAL(&s_stream_mux);
  iq_network_stream_set_format(format);
  iq_network_stream_begin(IQ_STREAM_OWNER_ETH);
  esp_err_t response = send_status(req);
  iq_network_stream_arm();
  return response;
}

static esp_err_t stream_stop_handler(httpd_req_t *req) {
  iq_network_stream_end();
  return send_status(req);
}

iq_stream_owner_t iq_network_stream_owner(void) { return s_stream_owner; }

void iq_network_stream_set_format(uint32_t format) {
  taskENTER_CRITICAL(&s_stream_mux);
  s_stream_format = format;
  taskEXIT_CRITICAL(&s_stream_mux);
}

uint32_t iq_network_stream_format(void) { return s_stream_format; }

void iq_network_stream_begin(iq_stream_owner_t owner) {
  taskENTER_CRITICAL(&s_stream_mux);
  /* New start wins: replacing the owner silently stops the previous
   * transport because send_frame and tx_ticket check ownership. */
  s_stream_owner = owner;
  s_stream_armed = false;
  ++s_stream_epoch;
  if (s_stream_epoch == 0)
    ++s_stream_epoch;
  s_datagram_sequence = 1;
  taskEXIT_CRITICAL(&s_stream_mux);
}

void iq_network_stream_arm(void) {
  s_callbacks.set_capture_armed(true);
  (void)esp_timer_stop(s_stream_arm_timer);
  ESP_ERROR_CHECK(esp_timer_start_once(s_stream_arm_timer, STREAM_ARM_DELAY_US));
}

void iq_network_stream_end(void) {
  (void)esp_timer_stop(s_stream_arm_timer);
  taskENTER_CRITICAL(&s_stream_mux);
  s_stream_armed = false;
  s_stream_owner = IQ_STREAM_OWNER_NONE;
  taskEXIT_CRITICAL(&s_stream_mux);
  s_callbacks.set_capture_armed(false);
}

bool iq_network_stream_tx_ticket(iq_stream_owner_t owner, uint32_t *epoch,
                                 uint32_t *sequence) {
  bool ok;
  taskENTER_CRITICAL(&s_stream_mux);
  ok = s_stream_armed && s_stream_owner == owner;
  if (ok) {
    *epoch = s_stream_epoch;
    *sequence = s_datagram_sequence++;
  }
  taskEXIT_CRITICAL(&s_stream_mux);
  return ok;
}

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");
extern const char espargos_logo_svg_start[]
    asm("_binary_espargos_logo_svg_start");
extern const char espargos_logo_svg_end[]
    asm("_binary_espargos_logo_svg_end");

static esp_err_t send_static_asset(httpd_req_t *req, const char *type,
                                   const char *cache_control,
                                   const char *start, const char *end) {
  httpd_resp_set_type(req, type);
  httpd_resp_set_hdr(req, "Cache-Control", cache_control);
  httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
  return httpd_resp_send(req, start, end - start - 1u);
}

static esp_err_t index_handler(httpd_req_t *req) {
  return send_static_asset(req, "text/html; charset=utf-8",
                           "no-cache", index_html_start, index_html_end);
}

static esp_err_t logo_handler(httpd_req_t *req) {
  return send_static_asset(req, "image/svg+xml", "public, max-age=3600",
                           espargos_logo_svg_start, espargos_logo_svg_end);
}

static void start_http_server(void) {
  if (s_http_server != NULL)
    return;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = IQ_NETWORK_HTTP_PORT;
  config.stack_size = 6144;
  /* TX staging, EMAC RX, and TinyUSB deliberately cooperate at priority 23.
   * The per-batch UDP arm must join that yield set: leaving HTTP at priority
   * 20 starves the continuation publish while the internal TX ring is full,
   * then drains the ring at every host-batch boundary. The server blocks when
   * idle, so this does not consume data-plane CPU between requests. */
  config.task_priority = 23;
  config.task_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
  /* Keep control off the dedicated core-1 PARLIO producer. At 40 MSa/s that
   * priority-24 producer is continuously runnable, so an unpinned HTTP task
   * can accept a TCP connection yet never execute its handler. */
  config.core_id = 0;
  config.max_uri_handlers = 7;
  ESP_ERROR_CHECK(httpd_start(&s_http_server, &config));
  const httpd_uri_t handlers[] = {
      {.uri = "/", .method = HTTP_GET, .handler = index_handler},
      {.uri = "/espargos-logo.svg",
       .method = HTTP_GET,
       .handler = logo_handler},
      {.uri = "/api/v1/status",
       .method = HTTP_GET,
       .handler = status_get_handler},
      {.uri = "/api/v1/config",
       .method = HTTP_GET,
       .handler = config_get_handler},
      {.uri = "/api/v1/config",
       .method = HTTP_PUT,
       .handler = config_put_handler},
      {.uri = "/api/v1/stream/start",
       .method = HTTP_POST,
       .handler = stream_start_handler},
      {.uri = "/api/v1/stream/stop",
       .method = HTTP_POST,
       .handler = stream_stop_handler},
  };
  for (size_t i = 0; i < sizeof(handlers) / sizeof(handlers[0]); ++i) {
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &handlers[i]));
  }
}

static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id,
                              void *data) {
  (void)arg;
  (void)base;
  (void)data;
  if (id == ETHERNET_EVENT_CONNECTED) {
    s_link_up = true;
    eth_speed_t speed = ETH_SPEED_10M;
    (void)esp_eth_ioctl(s_eth_handle, ETH_CMD_G_SPEED, &speed);
    ESP_LOGI(TAG, "Ethernet link up at %u Mbit/s",
             speed == ETH_SPEED_10M    ? 10u
             : speed == ETH_SPEED_100M ? 100u
                                       : 1000u);
  }
  if (id == ETHERNET_EVENT_DISCONNECTED || id == ETHERNET_EVENT_STOP) {
    ESP_LOGI(TAG, "Ethernet link down");
    s_link_up = false;
    s_has_ipv4 = false;
    s_ipv4_address = 0;
  }
}

static void got_ip_handler(void *arg, esp_event_base_t base, int32_t id,
                           void *data) {
  (void)arg;
  (void)base;
  (void)id;
  ip_event_got_ip_t *event = data;
  s_ipv4_address = event->ip_info.ip.addr;
  s_has_ipv4 = true;
  ESP_LOGI(TAG, "DHCP IPv4 address: " IPSTR, IP2STR(&event->ip_info.ip));
  start_http_server();
}

void iq_control_init(const iq_network_callbacks_t *callbacks) {
  s_callbacks = *callbacks;
  s_stream_epoch = esp_random();
  if (s_stream_epoch == 0)
    s_stream_epoch = 1;

  const esp_timer_create_args_t stream_arm_timer_args = {
      .callback = stream_arm_timer_callback,
      .name = "iq_stream_arm",
  };
  ESP_ERROR_CHECK(
      esp_timer_create(&stream_arm_timer_args, &s_stream_arm_timer));
}

void iq_network_init(const iq_network_callbacks_t *callbacks) {
  iq_control_init(callbacks);

  esp_err_t eth_err = ethernet_driver_init();
  if (eth_err != ESP_OK) {
    ESP_LOGW(TAG, "Ethernet unavailable (%s); continuing without network",
             esp_err_to_name(eth_err));
    return;
  }
  esp_netif_config_t config = ESP_NETIF_DEFAULT_ETH();
  s_eth_netif = esp_netif_new(&config);
  ESP_ERROR_CHECK(s_eth_netif != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(s_eth_handle);
  ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, glue));
  ESP_ERROR_CHECK(esp_eth_update_input_path_info(
      s_eth_handle, iq_eth_input, s_eth_netif));
  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                             eth_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                             got_ip_handler, NULL));
  ESP_ERROR_CHECK(mdns_init());
  ESP_ERROR_CHECK(mdns_hostname_set(IQ_NETWORK_HOSTNAME));
  ESP_ERROR_CHECK(mdns_instance_name_set("ESP-SDR"));
  ESP_ERROR_CHECK(mdns_service_add("ESP-SDR HTTP", "_http", "_tcp",
                                   IQ_NETWORK_HTTP_PORT, NULL, 0));
  ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
  ESP_LOGI(TAG, "Ethernet started; waiting for link and DHCP");
}

bool iq_network_stream_armed(void) { return s_stream_armed; }

bool iq_network_send_frames(const uint8_t *const *frames,
                            const size_t *frame_sizes,
                            uint32_t frame_count) {
  if (frames == NULL || frame_sizes == NULL || frame_count == 0u ||
      frame_count > CONFIG_ETH_DMA_TX_BUFFER_NUM) {
    return false;
  }
  uint8_t fragment_counts[CONFIG_ETH_DMA_TX_BUFFER_NUM];
  uint32_t total_fragments = 0u;
  for (uint32_t frame_index = 0u; frame_index < frame_count; ++frame_index) {
    if (frames[frame_index] == NULL || frame_sizes[frame_index] < 8u) {
      return false;
    }
    uint32_t fragments =
        (frame_sizes[frame_index] + IQ_UDP_FRAGMENT_PAYLOAD_BYTES - 1u) /
        IQ_UDP_FRAGMENT_PAYLOAD_BYTES;
    if (fragments == 0u || fragments > UINT8_MAX ||
        total_fragments + fragments > CONFIG_ETH_DMA_TX_BUFFER_NUM) {
      return false;
    }
    fragment_counts[frame_index] = (uint8_t)fragments;
    total_fragments += fragments;
  }
  struct sockaddr_in destination;
  uint8_t destination_mac[6];
  uint32_t epoch;
  taskENTER_CRITICAL(&s_stream_mux);
  bool armed = s_stream_armed && s_stream_owner == IQ_STREAM_OWNER_ETH;
  destination = s_stream_destination;
  memcpy(destination_mac, s_stream_destination_mac, sizeof(destination_mac));
  epoch = s_stream_epoch;
  taskEXIT_CRITICAL(&s_stream_mux);
  if (!armed || !s_link_up || !s_has_ipv4)
    return false;

  iq_eth_driver_prefix_t *driver = (iq_eth_driver_prefix_t *)s_eth_handle;
  if (xSemaphoreTake(driver->transmit_mutex,
                     pdMS_TO_TICKS(IQ_ETH_TX_MUTEX_TIMEOUT_MS)) == pdFALSE) {
    ++s_udp_send_errors;
    return false;
  }
  iq_emac_mac_prefix_t *emac = (iq_emac_mac_prefix_t *)driver->mac;
  iq_emac_dma_prefix_t *dma = emac->emac_dma_hndl;
  eth_dma_tx_descriptor_t *first_desc = dma->tx_desc;
  eth_dma_tx_descriptor_t *desc = first_desc;
  uint32_t descriptor_deadline = esp_cpu_get_cycle_count() + 80000000u;
  uint32_t scheduler_yield_deadline =
      esp_cpu_get_cycle_count() + 320000u;
  while (true) {
    bool available = true;
    desc = first_desc;
    for (uint32_t i = 0u; i < total_fragments; ++i) {
      (void)esp_cache_msync(desc, EMAC_HAL_DMA_DESC_SIZE,
                            ESP_CACHE_MSYNC_FLAG_DIR_M2C);
      available &= desc->TDES0.Own == EMAC_LL_DMADESC_OWNER_CPU;
      desc = (eth_dma_tx_descriptor_t *)desc->Buffer2NextDescAddr;
    }
    if (available)
      break;
    /* A stop request runs in the higher-priority HTTP task. Drop the
     * in-progress, not-yet-owned descriptor batch immediately so the normal
     * Ethernet driver can acquire its mutex and transmit the stop response. */
    if (!s_stream_armed || s_stream_owner != IQ_STREAM_OWNER_ETH) {
      xSemaphoreGive(driver->transmit_mutex);
      return false;
    }
    if ((int32_t)(esp_cpu_get_cycle_count() - descriptor_deadline) >= 0) {
      ++s_udp_send_errors;
      xSemaphoreGive(driver->transmit_mutex);
      return false;
    }
    /* The GMAC enters TX-buffer-unavailable when it catches the CPU-owned
     * tail of the short descriptor ring.  Re-kick it while earlier
     * descriptors complete. Normal turnover takes microseconds; only block
     * a scheduler tick after one millisecond of continuous pressure so HTTP
     * stop/control remains responsive during a genuine overload. */
    emac_hal_transmit_poll_demand(&dma->hal);
    uint32_t now = esp_cpu_get_cycle_count();
    if ((int32_t)(now - scheduler_yield_deadline) >= 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      scheduler_yield_deadline = esp_cpu_get_cycle_count() + 320000u;
    } else {
      esp_rom_delay_us(2u);
    }
  }

  /* Every descriptor is inspected exactly once immediately before reuse.
   * The DMA replaces these status bits on each transmission, so the counters
   * describe completed descriptors rather than polling iterations. */
  desc = first_desc;
  for (uint32_t i = 0u; i < total_fragments; ++i) {
    ++s_eth_tx_desc_completed;
    if (desc->TDES0.ErrSummary != 0u)
      ++s_eth_tx_desc_errors;
    if (desc->TDES0.UnderflowErr != 0u)
      ++s_eth_tx_desc_underflows;
    if (desc->TDES0.FrameFlushed != 0u)
      ++s_eth_tx_desc_flushed;
    if (desc->TDES0.NoCarrier != 0u || desc->TDES0.LossCarrier != 0u)
      ++s_eth_tx_desc_carrier_errors;
    desc = (eth_dma_tx_descriptor_t *)desc->Buffer2NextDescAddr;
  }

  uint8_t *packet = s_tx_frame + ETH_HEADER_BYTES + IPV4_HEADER_BYTES +
                    UDP_HEADER_BYTES;
  iq_udp_header_t *header = (iq_udp_header_t *)packet;
  desc = first_desc;
  for (uint32_t frame_index = 0u; frame_index < frame_count; ++frame_index) {
    const uint8_t *frame = frames[frame_index];
    size_t frame_bytes = frame_sizes[frame_index];
    uint32_t frame_sequence;
    memcpy(&frame_sequence, frame + 4u, sizeof(frame_sequence));
    uint32_t source_chunk = 0u;
    uint32_t dropped = s_callbacks.get_firmware_dropped_chunks();
    bool iq_frame =
        memcmp(frame, STREAM_FRAME_MAGIC_IQ, 4u) == 0 ||
        memcmp(frame, STREAM_FRAME_MAGIC_IQ8, 4u) == 0 ||
        memcmp(frame, STREAM_FRAME_MAGIC_REAL8, 4u) == 0;
    if (iq_frame && frame_bytes >= 40u) {
      memcpy(&source_chunk, frame + 8u, sizeof(source_chunk));
      memcpy(&dropped, frame + 36u, sizeof(dropped));
    }
    uint32_t frame_crc;
    if (iq_frame) {
      memcpy(&frame_crc, frame + frame_bytes - sizeof(frame_crc),
             sizeof(frame_crc));
    } else {
      frame_crc = esp_rom_crc32_le(0u, frame, frame_bytes);
    }
    uint32_t fragment_count = fragment_counts[frame_index];
    for (uint32_t i = 0u, offset = 0u; i < fragment_count; ++i) {
      uint32_t remaining = frame_bytes - offset;
      uint16_t payload_bytes = remaining > IQ_UDP_FRAGMENT_PAYLOAD_BYTES
                                   ? IQ_UDP_FRAGMENT_PAYLOAD_BYTES
                                   : (uint16_t)remaining;
      *header = (iq_udp_header_t){
          .magic = {'I', 'Q', 'U', '1'},
          .version = IQ_UDP_VERSION,
          .header_bytes = sizeof(*header),
          .stream_epoch = epoch,
          .datagram_sequence = s_datagram_sequence,
          .frame_sequence = frame_sequence,
          .source_chunk_index = source_chunk,
          .frame_bytes = frame_bytes,
          .frame_crc32 = frame_crc,
          .fragment_offset = offset,
          .fragment_bytes = payload_bytes,
          .fragment_index = i,
          .fragment_count = fragment_count,
          .firmware_dropped_chunks = dropped,
      };
      memcpy(header->frame_magic, frame, 4u);
      header->header_crc32 = esp_rom_crc32_le(
          0u, packet, offsetof(iq_udp_header_t, header_crc32));
      size_t packet_bytes = sizeof(*header) + payload_bytes;
      build_udp_frame(&destination, destination_mac, packet_bytes,
                      (uint16_t)s_datagram_sequence);
      const uint32_t prefix_bytes =
          ETH_HEADER_BYTES + IPV4_HEADER_BYTES + UDP_HEADER_BYTES +
          sizeof(*header);
      uint8_t *dma_buffer = (uint8_t *)desc->Buffer1Addr;
      memcpy(dma_buffer, s_tx_frame, prefix_bytes);
      memcpy(dma_buffer + prefix_bytes, frame + offset, payload_bytes);
      uint32_t ethernet_bytes = prefix_bytes + payload_bytes;
      desc->TDES0.Value &=
          ~(IQ_EMAC_TDES0_FS_FLAGS_MASK | IQ_EMAC_TDES0_LS_FLAGS_MASK);
      desc->TDES0.FirstSegment = 1u;
      desc->TDES0.LastSegment = 1u;
      desc->TDES0.Value |=
          dma->tx_desc_flags &
          (IQ_EMAC_TDES0_FS_FLAGS_MASK | IQ_EMAC_TDES0_LS_FLAGS_MASK);
      desc->TDES1.TransmitBuffer1Size = ethernet_bytes;
      uint32_t cache_bytes = (ethernet_bytes + 63u) & ~63u;
      (void)esp_cache_msync(dma_buffer, cache_bytes,
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M);
      desc = (eth_dma_tx_descriptor_t *)desc->Buffer2NextDescAddr;
      ++s_datagram_sequence;
      ++s_udp_datagrams;
      s_udp_bytes += (uint32_t)packet_bytes;
      offset += payload_bytes;
    }
    ++s_udp_frames;
  }
  desc = first_desc;
  for (uint32_t i = 0u; i < total_fragments; ++i) {
    desc->TDES0.Own = EMAC_LL_DMADESC_OWNER_DMA;
    (void)esp_cache_msync(desc, EMAC_HAL_DMA_DESC_SIZE,
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    desc = (eth_dma_tx_descriptor_t *)desc->Buffer2NextDescAddr;
  }
  dma->tx_desc = desc;
  emac_hal_transmit_poll_demand(&dma->hal);
  s_eth_dma_status = emac_hal_get_intr_status(&dma->hal);
  xSemaphoreGive(driver->transmit_mutex);
  return true;
}

bool iq_network_send_frame(const uint8_t *frame, size_t frame_bytes) {
  const uint8_t *frames[1] = {frame};
  size_t frame_sizes[1] = {frame_bytes};
  return iq_network_send_frames(frames, frame_sizes, 1u);
}

bool iq_network_send_frame_int8(const uint8_t *frame, size_t frame_bytes) {
  if (frame == NULL || frame_bytes != sizeof(iq_chunk_t) ||
      memcmp(frame, STREAM_FRAME_MAGIC_IQ, 4u) != 0) {
    return false;
  }
  memcpy(s_iq8_frame, frame, offsetof(iq_chunk_t, samples));
  s_iq8_frame[3] = '8';
  const uint32_t *source = (const uint32_t *)(
      frame + offsetof(iq_chunk_t, samples));
  uint32_t *packed = (uint32_t *)(
      s_iq8_frame + offsetof(iq_chunk_t, samples));
  for (uint32_t sample = 0u; sample < IQ_CHUNK_SAMPLE_WORDS; sample += 2u) {
    uint32_t w0 = source[sample];
    uint32_t w1 = source[sample + 1u];
    uint32_t p0 = ((w0 >> 12) & 0xffu) | ((w0 << 6) & 0xff00u);
    uint32_t p1 = ((w1 >> 12) & 0xffu) | ((w1 << 6) & 0xff00u);
    packed[sample / 2u] = p0 | (p1 << 16);
  }
  memset(s_iq8_frame + IQ8_FRAME_WIRE_BYTES - sizeof(uint32_t), 0,
         sizeof(uint32_t));
  return iq_network_send_frame(s_iq8_frame, sizeof(s_iq8_frame));
}
