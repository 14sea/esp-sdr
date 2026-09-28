#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"

#define IQ_NETWORK_HOSTNAME "esp-sdr"
#define IQ_NETWORK_HTTP_PORT 80u
#define IQ_NETWORK_UDP_DEFAULT_PORT 50000u
#define IQ_RX_BASE_SAMPLE_RATE_HZ 16000000u
#define IQ_UDP_MAGIC "IQU1"
#define IQ_UDP_VERSION 1u
#define IQ_UDP_FRAGMENT_PAYLOAD_BYTES 1400u

typedef struct __attribute__((packed)) {
  char magic[4];
  uint16_t version;
  uint16_t header_bytes;
  uint32_t stream_epoch;
  uint32_t datagram_sequence;
  uint32_t frame_sequence;
  uint32_t source_chunk_index;
  uint32_t frame_bytes;
  uint32_t frame_crc32;
  uint32_t fragment_offset;
  uint16_t fragment_bytes;
  uint8_t fragment_index;
  uint8_t fragment_count;
  char frame_magic[4];
  uint32_t firmware_dropped_chunks;
  uint32_t header_crc32;
} iq_udp_header_t;

_Static_assert(sizeof(iq_udp_header_t) == 52u,
               "IQ UDP header wire size changed");

typedef void (*iq_network_get_config_fn)(capture_config_t *config);
typedef void (*iq_network_apply_config_fn)(const capture_config_t *config);
typedef uint32_t (*iq_network_get_u32_fn)(void);
typedef bool (*iq_network_get_bool_fn)(void);
typedef void (*iq_network_set_bool_fn)(bool value);
typedef struct {
  iq_network_get_config_fn get_config;
  iq_network_apply_config_fn apply_config;
  iq_network_get_u32_fn get_firmware_dropped_chunks;
  iq_network_get_u32_fn get_source_chunk_index;
  /* Live ADC dump registers, exposed for selector/routing diagnostics. */
  iq_network_get_u32_fn get_adc_dump_cfg;
  iq_network_get_u32_fn get_adc_dump_mode;
  iq_network_get_u32_fn get_modem_diag_fix_sel;
  iq_network_get_u32_fn get_modem_diag_exchange;
  iq_network_get_u32_fn get_bb_diag;
  /* DCOC servo state word: bit 31 = active, bits 14-27 / 0-13 = last I/Q DC
   * estimate (signed 14-bit, ADC counts). */
  iq_network_get_u32_fn get_dcoc_diag;
  iq_network_get_bool_fn get_dcoc_active;
  iq_network_get_bool_fn is_config_applying;
  iq_network_set_bool_fn set_capture_armed;
} iq_network_callbacks_t;

/* Initialize the control plane and shared stream-owner state without starting
 * a physical transport.  USB-only builds need this even though they do not
 * install the Ethernet driver. */
void iq_control_init(const iq_network_callbacks_t *callbacks);
void iq_network_init(const iq_network_callbacks_t *callbacks);
bool iq_network_send_frame(const uint8_t *frame, size_t frame_bytes);
bool iq_network_send_frames(const uint8_t *const *frames,
                            const size_t *frame_bytes, uint32_t frame_count);
/* Quantize an IQC1 frame to IQC8 in the normal transport context, then send
 * it. This keeps S31 capture transactions on their proven full-word path. */
bool iq_network_send_frame_int8(const uint8_t *frame, size_t frame_bytes);
bool iq_network_stream_armed(void);

/* The sample stream has one owner at a time; a start on either transport
 * replaces the current stream (new start wins). Control stays available on
 * both transports throughout. */
typedef enum {
  IQ_STREAM_OWNER_NONE = 0,
  IQ_STREAM_OWNER_ETH = 1,
  IQ_STREAM_OWNER_USB = 2,
} iq_stream_owner_t;

iq_stream_owner_t iq_network_stream_owner(void);
/* Wire format selected by the active stream-start request.  Values use the
 * IQ_USB_FORMAT_* enum because USB and Ethernet share IQC1/IQC8 framing. */
void iq_network_stream_set_format(uint32_t format);
uint32_t iq_network_stream_format(void);
/* Claim the stream: bumps the epoch and resets the datagram sequence. */
void iq_network_stream_begin(iq_stream_owner_t owner);
/* Arm capture and start the delayed stream-armed timer. */
void iq_network_stream_arm(void);
/* Stop streaming and release ownership (any owner). */
void iq_network_stream_end(void);
/* Fetch the epoch and allocate the next datagram sequence number for one
 * outgoing message. Fails unless armed and owned by `owner`. */
bool iq_network_stream_tx_ticket(iq_stream_owner_t owner, uint32_t *epoch,
                                 uint32_t *sequence);

/* Shared control-plane helpers (JSON bodies identical on HTTP and USB). */
struct cJSON;
int iq_control_build_config_json(char *buf, size_t cap);
int iq_control_build_status_json(char *buf, size_t cap);
/* Merge a parsed JSON patch onto the current configuration and validate.
 * Returns NULL on success (with *config filled in), else an error string. */
const char *iq_control_parse_config_json(struct cJSON *root,
                                         capture_config_t *config);
