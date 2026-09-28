#include "ringbuffer.h"

#include "dcoc.h"

#include <assert.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"

static portMUX_TYPE s_stream_ring_mux = portMUX_INITIALIZER_UNLOCKED;
/* Wakes the stream task as soon as a frame is committed. The USB transport
 * drains barely faster than full-rate frames arrive, so a fixed 1 ms sleep on
 * an empty ring would never catch up and the source would drop chunks. */
static SemaphoreHandle_t s_stream_ring_frames_sem;
static stream_frame_t *s_stream_ring[IQ_STREAM_RING_CHUNKS];
static stream_frame_t *s_stream_ring_expected[IQ_STREAM_RING_CHUNKS];
static volatile uint32_t s_stream_ring_read;
static volatile uint32_t s_stream_ring_write;
static volatile uint32_t s_stream_ring_count;
static volatile uint32_t s_stream_ring_reserved;
static volatile uint32_t s_stream_sequence = 1;
static bool s_iq_timestamp_valid;
static uint32_t s_iq_timestamp_last_source;
static uint32_t s_iq_timestamp_last_rate_hz;
static uint64_t s_iq_timestamp_last_us;

static uint64_t IRAM_ATTR iq_timestamp_us(
    const iq_chunk_report_meta_t *meta) {
  const uint64_t now_us = (uint64_t)esp_timer_get_time();
  if (meta->sample_rate_hz == 0u) {
    return now_us;
  }
  const uint64_t chunk_us =
      ((uint64_t)IQ_CHUNK_SAMPLE_WORDS * 1000000u +
       meta->sample_rate_hz / 2u) /
      meta->sample_rate_hz;
  uint64_t timestamp_us = now_us;
  if (s_iq_timestamp_valid) {
    const uint32_t source_delta =
        meta->source_chunk_index - s_iq_timestamp_last_source;
    const bool source_forward =
        source_delta != 0u && source_delta < 0x80000000u;
    if (source_forward &&
        meta->sample_rate_hz == s_iq_timestamp_last_rate_hz) {
      timestamp_us = s_iq_timestamp_last_us +
                     ((uint64_t)source_delta * IQ_CHUNK_SAMPLE_WORDS *
                          1000000u +
                      meta->sample_rate_hz / 2u) /
                         meta->sample_rate_hz;
      /* A capture pause can leave the source counter frozen. Re-anchor only
       * for a material wall-clock discontinuity; ordinary batching may build
       * several headers ahead of their capture and must keep sample cadence. */
      if (now_us > timestamp_us + chunk_us * 8u) {
        timestamp_us = now_us;
      }
    } else {
      const uint64_t next_us = s_iq_timestamp_last_us + chunk_us;
      if (timestamp_us < next_us) {
        timestamp_us = next_us;
      }
    }
  }
  s_iq_timestamp_valid = true;
  s_iq_timestamp_last_source = meta->source_chunk_index;
  s_iq_timestamp_last_rate_hz = meta->sample_rate_hz;
  s_iq_timestamp_last_us = timestamp_us;
  return timestamp_us;
}

static inline void stream_ring_check_slot(uint32_t index) {
  assert(index < IQ_STREAM_RING_CHUNKS);
  assert(s_stream_ring[index] != NULL);
  assert(s_stream_ring[index] == s_stream_ring_expected[index]);
}

/* All producers commit from task context. */
static void stream_ring_signal_frames(uint32_t count) {
  if (s_stream_ring_frames_sem == NULL) {
    return;
  }
  for (uint32_t i = 0u; i < count; ++i) {
    (void)xSemaphoreGive(s_stream_ring_frames_sem);
  }
}

bool stream_ring_wait_frames(uint32_t timeout_ms) {
  if (s_stream_ring_frames_sem == NULL) {
    return false;
  }
  return xSemaphoreTake(s_stream_ring_frames_sem, pdMS_TO_TICKS(timeout_ms)) ==
         pdTRUE;
}

void stream_ring_init_storage(void) {
  if (s_stream_ring[0] != NULL) {
    return;
  }
  s_stream_ring_frames_sem =
      xSemaphoreCreateCounting(IQ_STREAM_RING_CHUNKS, 0u);
  ESP_ERROR_CHECK(s_stream_ring_frames_sem != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  for (uint32_t i = 0u; i < IQ_STREAM_RING_CHUNKS; ++i) {
    s_stream_ring[i] = heap_caps_calloc(1u, sizeof(*s_stream_ring[i]),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_ERROR_CHECK(s_stream_ring[i] != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    s_stream_ring_expected[i] = s_stream_ring[i];
  }
}

void stream_ring_reset(void) {
  taskENTER_CRITICAL(&s_stream_ring_mux);
  s_stream_ring_read = 0u;
  s_stream_ring_write = 0u;
  s_stream_ring_count = 0u;
  s_stream_ring_reserved = 0u;
  taskEXIT_CRITICAL(&s_stream_ring_mux);
}

bool IRAM_ATTR stream_ring_reserve_slots(uint32_t count,
                                         stream_frame_t **slots) {
  if (count == 0u || count > IQ_STREAM_RING_CHUNKS) {
    return false;
  }
  bool reserved = false;
  taskENTER_CRITICAL(&s_stream_ring_mux);
  uint32_t free_chunks =
      IQ_STREAM_RING_CHUNKS - s_stream_ring_count - s_stream_ring_reserved;
  if (free_chunks >= count) {
    for (uint32_t i = 0u; i < count; ++i) {
      uint32_t index = (s_stream_ring_write + i) % IQ_STREAM_RING_CHUNKS;
      stream_ring_check_slot(index);
      slots[i] = s_stream_ring[index];
    }
    s_stream_ring_write = (s_stream_ring_write + count) % IQ_STREAM_RING_CHUNKS;
    s_stream_ring_reserved += count;
    reserved = true;
  }
  taskEXIT_CRITICAL(&s_stream_ring_mux);
  return reserved;
}

void IRAM_ATTR stream_ring_commit_reserved(uint32_t count) {
  if (count == 0u) {
    return;
  }
  taskENTER_CRITICAL(&s_stream_ring_mux);
  if (count > s_stream_ring_reserved) {
    count = s_stream_ring_reserved;
  }
  s_stream_ring_reserved -= count;
  s_stream_ring_count += count;
  taskEXIT_CRITICAL(&s_stream_ring_mux);
  stream_ring_signal_frames(count);
}

void IRAM_ATTR stream_ring_release_reserved(uint32_t count) {
  if (count == 0u) {
    return;
  }
  taskENTER_CRITICAL(&s_stream_ring_mux);
  if (count > s_stream_ring_reserved) {
    count = s_stream_ring_reserved;
  }
  s_stream_ring_reserved -= count;
  s_stream_ring_write = (s_stream_ring_write + IQ_STREAM_RING_CHUNKS - count) %
                        IQ_STREAM_RING_CHUNKS;
  taskEXIT_CRITICAL(&s_stream_ring_mux);
}

uint32_t IRAM_ATTR stream_ring_available_slots(void) {
  uint32_t available;
  taskENTER_CRITICAL(&s_stream_ring_mux);
  available =
      IQ_STREAM_RING_CHUNKS - s_stream_ring_count - s_stream_ring_reserved;
  taskEXIT_CRITICAL(&s_stream_ring_mux);
  return available;
}

bool stream_ring_push_wifi(const wifi_packet_report_t *report) {
  bool pushed = false;
  taskENTER_CRITICAL(&s_stream_ring_mux);
  if (s_stream_ring_reserved == 0u &&
      s_stream_ring_count < IQ_STREAM_RING_CHUNKS) {
    stream_ring_check_slot(s_stream_ring_write);
    s_stream_ring[s_stream_ring_write]->wifi = *report;
    s_stream_ring_write = (s_stream_ring_write + 1u) % IQ_STREAM_RING_CHUNKS;
    ++s_stream_ring_count;
    pushed = true;
  }
  taskEXIT_CRITICAL(&s_stream_ring_mux);
  if (pushed) {
    stream_ring_signal_frames(1u);
  }
  return pushed;
}

bool stream_ring_push_config_report(const config_report_t *report) {
  bool pushed = false;
  taskENTER_CRITICAL(&s_stream_ring_mux);
  if (s_stream_ring_reserved == 0u &&
      s_stream_ring_count < IQ_STREAM_RING_CHUNKS) {
    stream_ring_check_slot(s_stream_ring_write);
    stream_frame_t *frame = s_stream_ring[s_stream_ring_write];
    frame->config = *report;
    frame->config.sequence = s_stream_sequence++;
    s_stream_ring_write = (s_stream_ring_write + 1u) % IQ_STREAM_RING_CHUNKS;
    ++s_stream_ring_count;
    pushed = true;
  }
  taskEXIT_CRITICAL(&s_stream_ring_mux);
  if (pushed) {
    stream_ring_signal_frames(1u);
  }
  return pushed;
}

static void IRAM_ATTR fill_iq_header(stream_frame_t *frame,
                                     const iq_chunk_report_meta_t *meta) {
  memcpy(frame->iq.magic, STREAM_FRAME_MAGIC_IQ, sizeof(frame->iq.magic));
  frame->iq.sequence = s_stream_sequence++;
  frame->iq.source_chunk_index = meta->source_chunk_index;
  frame->iq.chunk_counter = meta->sample_rate_hz != 0u
                                ? (uint32_t)iq_timestamp_us(meta)
                                : meta->source_chunk_index;
  frame->iq.adc_decimation = meta->adc_decimation;
  frame->iq.sample_rate_hz = meta->sample_rate_hz;
  frame->iq.center_freq_mhz = meta->center_freq_mhz;
  frame->iq.rx_gain = meta->rx_gain;
  frame->iq.flags = meta->agc_state & STREAM_FRAME_AGC_STATE_MASK;
  bool dcoc_running = dcoc_active();
  if (dcoc_running) {
    /* Carry the DCOC estimator's signed residuals without adding wire bytes. */
    uint32_t diagnostic = dcoc_diag();
    uint32_t raw_i = (diagnostic >> 14) & 0x3fffu;
    uint32_t raw_q = diagnostic & 0x3fffu;
    int32_t error_i = (raw_i & 0x2000u) != 0u
                          ? (int32_t)(raw_i | 0xffffc000u)
                          : (int32_t)raw_i;
    int32_t error_q = (raw_q & 0x2000u) != 0u
                          ? (int32_t)(raw_q | 0xffffc000u)
                          : (int32_t)raw_q;
    error_i = error_i < -2048 ? -2048 : (error_i > 2047 ? 2047 : error_i);
    error_q = error_q < -2048 ? -2048 : (error_q > 2047 ? 2047 : error_q);
    frame->iq.flags |=
        ((uint32_t)error_i & STREAM_FRAME_DCOC_ERROR_M)
        << STREAM_FRAME_DCOC_ERROR_I_S;
    frame->iq.flags |=
        ((uint32_t)error_q & STREAM_FRAME_DCOC_ERROR_M)
        << STREAM_FRAME_DCOC_ERROR_Q_S;
  }
  if (dcoc_running) {
    frame->iq.flags |= STREAM_FRAME_FLAG_DCOC_ACTIVE;
  }
  if (meta->sample_rate_hz != 0u) {
    frame->iq.flags |= STREAM_FRAME_FLAG_TIMESTAMP_US32;
  }
  frame->iq.dropped_chunks = meta->dropped_chunks;
  frame->iq.bank_timer_late_misses = meta->bank_timer_late_misses;
  frame->iq.bank_timer_write_ptr = meta->bank_timer_write_ptr;
  frame->iq.producer_wake_write_ptr = meta->producer_wake_write_ptr;
}

void IRAM_ATTR stream_ring_fill_iq_chunk(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *source_words) {
  fill_iq_header(frame, meta);
  for (uint32_t i = 0u; i < IQ_CHUNK_SAMPLE_WORDS; ++i) {
    frame->iq.samples[i] = source_words[i];
  }
  frame->iq.crc32 = 0u;
}

void IRAM_ATTR stream_ring_prepare_iq_chunk(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta) {
  fill_iq_header(frame, meta);
  frame->iq.crc32 = 0u;
}

void IRAM_ATTR stream_ring_fill_iq_chunk_strided(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *ring_words, uint32_t start_word,
    uint32_t stride_words, uint32_t ring_word_count) {
  fill_iq_header(frame, meta);
  uint32_t source = start_word;
  for (uint32_t i = 0u; i < IQ_CHUNK_SAMPLE_WORDS; ++i) {
    frame->iq.samples[i] = ring_words[source];
    source += stride_words;
    if (source >= ring_word_count)
      source -= ring_word_count;
  }
  frame->iq.crc32 = 0u;
}

/* Per sample the low byte is the I MSBs (dump word bits 19:12) and the
 * high byte the Q MSBs (bits 9:2); two samples per 32-bit store. */
static inline uint32_t pack_iq8_pair(uint32_t w0, uint32_t w1) {
  const uint32_t p0 = ((w0 >> 12) & 0xffu) | ((w0 << 6) & 0xff00u);
  const uint32_t p1 = ((w1 >> 12) & 0xffu) | ((w1 << 6) & 0xff00u);
  return p0 | (p1 << 16);
}

/* frame is 4-byte aligned and samples sit at packed offset 52, so the
 * uint32 view is aligned despite the packed attribute. */
static inline uint32_t *iq8_packed_words(stream_frame_t *frame) {
  return (uint32_t *)((uint8_t *)frame + offsetof(iq_chunk_t, samples));
}

static inline void finish_iq8_frame(stream_frame_t *frame) {
  frame->iq.magic[3] = '8';
  /* Wire CRC field directly after the 2048 packed bytes; kept zero like
   * the Ethernet-only IQ frames. */
  iq8_packed_words(frame)[IQ_CHUNK_SAMPLE_WORDS / 2u] = 0u;
}

void IRAM_ATTR stream_ring_fill_iq_chunk_int8(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *source_words) {
  fill_iq_header(frame, meta);
  uint32_t *packed = iq8_packed_words(frame);
  for (uint32_t i = 0u; i < IQ_CHUNK_SAMPLE_WORDS; i += 2u) {
    packed[i >> 1] = pack_iq8_pair(source_words[i], source_words[i + 1u]);
  }
  finish_iq8_frame(frame);
}

static inline uint32_t pack_bt_bytes_pair(uint32_t w0, uint32_t w1) {
  return (w0 & 0xffffu) | ((w1 & 0xffffu) << 16);
}

void IRAM_ATTR stream_ring_fill_bt_bytes_chunk_int8(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *source_words) {
  fill_iq_header(frame, meta);
  uint32_t *packed = iq8_packed_words(frame);
  for (uint32_t i = 0u; i < IQ_CHUNK_SAMPLE_WORDS; i += 2u) {
    packed[i >> 1] =
        pack_bt_bytes_pair(source_words[i], source_words[i + 1u]);
  }
  finish_iq8_frame(frame);
}

void IRAM_ATTR stream_ring_fill_iq_chunk_strided_int8(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *ring_words, uint32_t start_word,
    uint32_t stride_words, uint32_t ring_word_count) {
  fill_iq_header(frame, meta);
  uint32_t *packed = iq8_packed_words(frame);
  uint32_t source = start_word;
  for (uint32_t i = 0u; i < IQ_CHUNK_SAMPLE_WORDS; i += 2u) {
    const uint32_t w0 = ring_words[source];
    source += stride_words;
    if (source >= ring_word_count)
      source -= ring_word_count;
    const uint32_t w1 = ring_words[source];
    source += stride_words;
    if (source >= ring_word_count)
      source -= ring_word_count;
    packed[i >> 1] = pack_iq8_pair(w0, w1);
  }
  finish_iq8_frame(frame);
}

void IRAM_ATTR stream_ring_fill_bt_bytes_chunk_strided_int8(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *ring_words, uint32_t start_word,
    uint32_t stride_words, uint32_t ring_word_count) {
  fill_iq_header(frame, meta);
  uint32_t *packed = iq8_packed_words(frame);
  uint32_t source = start_word;
  for (uint32_t i = 0u; i < IQ_CHUNK_SAMPLE_WORDS; i += 2u) {
    uint32_t w0 = ring_words[source];
    source += stride_words;
    if (source >= ring_word_count) source -= ring_word_count;
    uint32_t w1 = ring_words[source];
    source += stride_words;
    if (source >= ring_word_count) source -= ring_word_count;
    packed[i >> 1] = pack_bt_bytes_pair(w0, w1);
  }
  finish_iq8_frame(frame);
}

stream_frame_t *stream_ring_peek(void) {
  stream_frame_t *frame = NULL;
  taskENTER_CRITICAL(&s_stream_ring_mux);
  if (s_stream_ring_count != 0u) {
    stream_ring_check_slot(s_stream_ring_read);
    frame = s_stream_ring[s_stream_ring_read];
  }
  taskEXIT_CRITICAL(&s_stream_ring_mux);
  return frame;
}

uint32_t stream_ring_peek_batch(stream_frame_t **frames, uint32_t max_count) {
  if (frames == NULL || max_count == 0u) {
    return 0u;
  }
  uint32_t count;
  taskENTER_CRITICAL(&s_stream_ring_mux);
  count = s_stream_ring_count < max_count ? s_stream_ring_count : max_count;
  for (uint32_t i = 0u; i < count; ++i) {
    uint32_t index = (s_stream_ring_read + i) % IQ_STREAM_RING_CHUNKS;
    stream_ring_check_slot(index);
    frames[i] = s_stream_ring[index];
  }
  taskEXIT_CRITICAL(&s_stream_ring_mux);
  return count;
}

void stream_ring_pop_batch(uint32_t count) {
  if (count == 0u) {
    return;
  }
  taskENTER_CRITICAL(&s_stream_ring_mux);
  if (count > s_stream_ring_count) {
    count = s_stream_ring_count;
  }
  if (count == 0u) {
    taskEXIT_CRITICAL(&s_stream_ring_mux);
    return;
  }
  s_stream_ring_read = (s_stream_ring_read + count) % IQ_STREAM_RING_CHUNKS;
  s_stream_ring_count -= count;
  taskEXIT_CRITICAL(&s_stream_ring_mux);
}

void stream_ring_pop(void) { stream_ring_pop_batch(1u); }

size_t stream_frame_wire_size(stream_frame_t *frame) {
  if (memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_IQ, 4u) == 0) {
    frame->iq.crc32 = 0u;
    return sizeof(frame->iq);
  }
  if (memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_IQ8, 4u) == 0) {
    return IQ8_FRAME_WIRE_BYTES;
  }
  if (memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_IQ4, 4u) == 0) {
    return IQ4_FRAME_WIRE_BYTES;
  }
  if (memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_REAL8, 4u) == 0) {
    return REAL8_FRAME_WIRE_BYTES;
  }
  if (memcmp(frame->wifi.magic, WIFI_RX_REPORT_MAGIC, 4u) == 0) {
    return sizeof(frame->wifi);
  }
  if (memcmp(frame->config.magic, STREAM_FRAME_MAGIC_CONFIG, 4u) == 0) {
    frame->config.crc32 = esp_rom_crc32_le(0u, (const uint8_t *)&frame->config,
                                           offsetof(config_report_t, crc32));
    return sizeof(frame->config);
  }
  return 0u;
}
