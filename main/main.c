/* Continuous IQ capture firmware for the ESP32-S31 mac-dump engine. */
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/parlio_rx.h"
#include "esp_async_memcpy.h"
#include "esp_cache.h"
#include "esp_check.h"
#include "esp_clk_tree.h"
#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_freertos_hooks.h"
#include "esp_heap_caps.h"
#include "esp_ipc_isr.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_netif.h"
#include "esp_phy_cert_test.h"
#include "esp_private/esp_clk.h"
#include "esp_private/esp_gpio_reserve.h"
#include "esp_private/gdma.h"
#include "esp_private/gdma_link.h"
#include "esp_rom_crc.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/axi_dma_ll.h"
#include "hal/mwdt_ll.h"
#include "hal/parlio_ll.h"
#include "heap_memory_layout.h"
#include "modem/modem_widgets_reg.h"
#include "modem/reg_base.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "soc/ahb_dma_struct.h"
#include "soc/axi_dma_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/hp_system_reg.h"
#include "soc/lp_system_reg.h"
#include "soc/soc.h"
#include "soc/timer_group_struct.h"

#include "app_config.h"
#include "dcoc.h"
#include "iq_network.h"
#include "iq_usb.h"
#include "modem.h"
#include "phy_pbus_reg_names.h"
#include "phy_regs.h"
#include "reg_helpers.h"
#include "ringbuffer.h"
#include "wifi_rx.h"

#if CONFIG_IDF_TARGET_ESP32S31
extern void phy_pbus_force_test(uint32_t block, uint32_t bank,
                                uint32_t value);
extern uint32_t phy_pbus_rd(uint32_t block, uint32_t bank);
/*
 * S31 rftest's adctrig path and sentinel probes show the ADC dump engine writes
 * a fixed TCM aperture starting at 0x2f060000 in this mode. Reserving from
 * 0x2f050000 does not enlarge the ring; it keeps heap allocations out of a
 * guard area below the engine aperture so timing slips or misunderstood gate
 * effects are less likely to collide with CPU-owned objects.
 */
#define S31_MAC_DUMP_SRAM_GUARD_START 0x2f050000u
#define S31_ROM_RESERVED_TOP_START 0x2f07afb0u
#define S31_ROM_RESERVED_TOP_END 0x2f07f170u
#define S31_MAC_DUMP_SRAM_SAFE_END S31_ROM_RESERVED_TOP_START
SOC_RESERVE_MEMORY_REGION(S31_MAC_DUMP_SRAM_GUARD_START,
                          S31_MAC_DUMP_SRAM_SAFE_END, rf_adc_dump_window);
SOC_RESERVE_MEMORY_REGION(S31_ROM_RESERVED_TOP_START, S31_ROM_RESERVED_TOP_END,
                          s31_rom_reserved_top);
#else
SOC_RESERVE_MEMORY_REGION(MAC_DUMP_SRAM_BANK2_BASE, MAC_DUMP_SRAM_BANK2_END,
                          rf_adc_dump_bank2_window);
SOC_RESERVE_MEMORY_REGION(MAC_DUMP_SRAM_BANK3_BASE, MAC_DUMP_SRAM_BANK3_END,
                          rf_adc_dump_bank3_window);
#endif

/* Required by a linked component; unused here. */
int cmd_parse(char *cmd, char *name, int *argc, char **argv) {
  (void)cmd;
  (void)name;
  (void)argc;
  (void)argv;
  return -1;
}

/* Firmware policy: which mac-dump SRAM banks this firmware uses (per-bank
 * hardware facts live in phy_regs.h). BANK2+BANK3 as a contiguous window. */
#if CONFIG_IDF_TARGET_ESP32S31
#define MAC_DUMP_SRAM_USAGE MAC_DUMP_SRAM_BANK0_USAGE
#define MAC_DUMP_SRAM_BASE MAC_DUMP_SRAM_BANK0_BASE
#define MAC_DUMP_SRAM_WINDOW_BYTES                                             \
  (S31_MAC_DUMP_SRAM_SAFE_END - MAC_DUMP_SRAM_BANK0_BASE)
#else
#define MAC_DUMP_SRAM_USAGE                                                    \
  (MAC_DUMP_SRAM_BANK2_USAGE | MAC_DUMP_SRAM_BANK3_USAGE)
#define MAC_DUMP_SRAM_BASE MAC_DUMP_SRAM_BANK2_BASE
#define MAC_DUMP_SRAM_WINDOW_BYTES (2u * MAC_DUMP_SRAM_BANK_BYTES)
#endif

#define MAX_DUMP_WORDS (MAC_DUMP_SRAM_WINDOW_BYTES / sizeof(uint32_t))
#if CONFIG_IDF_TARGET_ESP32S31
#define DUMP_BANK_WORDS                                                        \
  ((MAX_DUMP_WORDS / IQ_CHUNK_SAMPLE_WORDS) * IQ_CHUNK_SAMPLE_WORDS)
#else
#define DUMP_BANK_WORDS MAC_DUMP_SRAM_BANK_WORDS /* 16384 */
#endif
_Static_assert(DUMP_BANK_WORDS <= MAX_DUMP_WORDS,
               "dump ring exceeds reserved SRAM");
_Static_assert((DUMP_BANK_WORDS % IQ_CHUNK_SAMPLE_WORDS) == 0u,
               "dump ring must be chunk-aligned");
#if CONFIG_IDF_TARGET_ESP32S31
/* Legacy S31 snapshot path. The production continuous real-IF backend below
 * bypasses this TCM dump source entirely. */
#define ADC_DUMP_SOURCE_CONTINUOUS_RX_IQ 0x49980003u
#else
#define ADC_DUMP_SOURCE_CONTINUOUS_RX_IQ 15u
#endif
#if CONFIG_IDF_TARGET_ESP32S31
#define S31_POISON_WORD 0xa5a0055au
#define S31_ADC_DUMP_SOURCE_SEL_MASK 0x0fu
#define S31_ADCTRIG_TCM_DUMP_CTRL HP_SYSTEM_TCM_DATA_DUMP_MAC_MASK
#define S31_ADC_SOURCE_PULSE_TCM_BEFORE_COPY BIT(19)
#define S31_ADC_SOURCE_TCM_ALWAYS_ON BIT(16)
#define S31_ADC_SOURCE_LONG_GATE_DWELL BIT(17)
#define S31_ADC_SOURCE_FIELD7_DWELL_X2 BIT(18)
#define S31_ADC_SOURCE_FIELD7_DWELL_X4 BIT(24)
#define S31_MODEM_DIAG_PROBE_OVERRIDE_FIRST 38u
#define S31_MODEM_DIAG_PROBE_OVERRIDE_LAST 47u
#define S31_MICRO_GATE_OVERRIDE 48u
#define S31_DATADUMP_MMIO_PROBE_OVERRIDE_FIRST 49u
#define S31_DATADUMP_MMIO_PROBE_OVERRIDE_LAST 55u
#define S31_LP_CORE_PROBE_OVERRIDE 56u
#define S31_HP_TCM_PROBE_OVERRIDE 57u
#define S31_SEAMLESS_MICRO_GATE_OVERRIDE 58u
#define S31_RING_CURSOR_OVERRIDE 59u
#define S31_GPIO_DIAG_OVERRIDE 60u
#define S31_PARLIO_NATIVE_IQ_OVERRIDE 61u
#define S31_PARLIO_HOST_IQ_OVERRIDE 62u
#define S31_PARLIO_BATCH_CHUNKS 14u
#define S31_CONTINUOUS_OUTPUT_CHUNKS (S31_PARLIO_BATCH_CHUNKS / 2u)
#define S31_CONTINUOUS_INPUT_RATE_HZ 4000000u
#define S31_CONTINUOUS_IF_HZ 1000000u
#define S31_PARLIO_DIAG_RATE_HZ 16000000u
#define S31_PARLIO_HOST_SYNC_40M BIT(31)
#define S31_PARLIO_NATIVE_PACKED_IQ BIT(22)
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
#define S31_DEFAULT_FILTER_BW_MHZ 16u
#else
#define S31_DEFAULT_FILTER_BW_MHZ RX_FILTER_BW_OPEN
#endif
#define S31_PARLIO_HOST_SYNC_RATE_HZ 40000000u
#if CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
#define S31_PARLIO_HOST_REAL_RATE_HZ 16000000u
#else
#define S31_PARLIO_HOST_REAL_RATE_HZ 32000000u
#endif
/* selector 22, byte exchange 2/f/2/3. The exhaustive Pluto census found the
 * coherent signed ADC sample in diagnostic bits 8:0 for this route. */
#define S31_CONTINUOUS_DIAG_MODE 0x32f90016u
#define S31_PARLIO_BATCH_BYTES                                              \
  (S31_PARLIO_BATCH_CHUNKS * IQ_CHUNK_SAMPLE_WORDS * sizeof(uint16_t))
#if CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
/* Eight complete banks provide 57 ms of raw acquisition elasticity. USB and
 * its transport consume the same average byte rate, so this absorbs
 * scheduling and endpoint jitter without copying in the ISR or surrendering
 * DMA ownership. */
#define S31_PARLIO_DMA_BYTES (8u * S31_PARLIO_BATCH_BYTES)
#else
#define S31_PARLIO_DMA_BYTES S31_PARLIO_BATCH_BYTES
#endif
#define S31_PARLIO_EVENT_RING_SIZE 128u
#define S31_PARLIO_EVENT_MAX_BYTES 4092u
/* The S31 driver uses 4032-byte cache-line-aligned GDMA nodes. Once this many
 * newer completions exist, the oldest node is already being reused. */
#define S31_PARLIO_DMA_NODE_BYTES 4032u
#define S31_PARLIO_DMA_NODE_COUNT                                           \
  ((S31_PARLIO_DMA_BYTES + S31_PARLIO_DMA_NODE_BYTES - 1u) /                \
   S31_PARLIO_DMA_NODE_BYTES)
/* Keep one complete descriptor between the node being copied and the GDMA
 * writer. Detecting only at NODE_COUNT is too late: GDMA may already have
 * started overwriting the oldest node by the time its predecessor reports
 * completion. */
#define S31_PARLIO_SAFE_NODE_BACKLOG (S31_PARLIO_DMA_NODE_COUNT - 1u)
/* Experimental pipelined snapshot backend.  The 64 KiB guard immediately
 * below the live aperture is used as an internal staging buffer so the dump
 * gate only has to be closed for a TCM-to-TCM copy. */
#define S31_ADC_SOURCE_PIPELINED_STAGE BIT(30)
#define S31_ADC_SOURCE_TCM_MASK_S 8u
#define S31_ADC_SOURCE_TCM_MASK_M (0xffu << S31_ADC_SOURCE_TCM_MASK_S)
#define S31_ADC_SOURCE_HW_DECIM_S 20u
#define S31_ADC_SOURCE_HW_DECIM_M (0xfu << S31_ADC_SOURCE_HW_DECIM_S)
#define S31_ADC_SOURCE_PULSE_DWELL_S 25u
#define S31_ADC_SOURCE_PULSE_DWELL_M (0x7u << S31_ADC_SOURCE_PULSE_DWELL_S)
#define S31_PULSE_META_MARKER 0x5a000000u
#endif
#define ADC_DECIMATION_MAX 10u
#define HZ_PER_MHZ 1000000u
/* ADC dump timing is derived from the 160 MHz modem clock, independently of
 * the HP CPU clock selected by CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ. */
#define ADC_DUMP_CLOCK_HZ 160000000u
#define STREAM_TASK_PRIORITY 18u
#define IQ_CHUNK_PRODUCER_TASK_STACK_BYTES 4096u
/* The raw S31 dump ring retains only about 333 us at 80 MSa/s.  Keep the
 * producer above ordinary system work on its dedicated core so snapshots
 * cannot be delayed past that hard deadline. */
#define IQ_CHUNK_PRODUCER_TASK_PRIORITY 24u
#define IQ_TASK_IDLE_DELAY_MS 2u

#define IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK 32u
#define IQ_AGC_TRIGGER_STATE_MASK_ALL 0xffffu
#define IQ_AGC_TRIGGER_MATCH_FSM 0u
#define IQ_AGC_TRIGGER_MATCH_MAX_GAIN 1u

enum {
  CHUNK_STREAM_SKIP = 0u,
  CHUNK_STREAM_TRIGGER = 1u,
  CHUNK_STREAM_PRE = 2u,
  CHUNK_STREAM_POST = 3u,
};

/* The dump window as a flat word array: word i is at physical bank i/16384,
 * offset i%16384. Per word: I=bits0-9, Q=bits10-19, rx_gain=20-27, agc=28-31.
 */
static volatile uint32_t *const DUMP_BASE =
    (volatile uint32_t *)MAC_DUMP_SRAM_BASE;

#if CONFIG_IDF_TARGET_ESP32S31
static void s31_enable_data_dump_clocks(const capture_config_t *config);
static void s31_configure_modem_diag(const capture_config_t *config);
static void s31_gpio_diag_prepare(const capture_config_t *config);
static void s31_gpio_diag_start(const capture_config_t *config);
static void s31_gpio_diag_stop(void);
static void s31_gpio_diag_release(void);
static void adctrig_prepare(uint32_t sample_count,
                            const capture_config_t *config);
static portMUX_TYPE s_tcm_dump_gate_mux;
static uint8_t *s_s31_parlio_dma_buffer;
#if CONFIG_ESP_SDR_TRANSPORT_USB && CONFIG_ESP_SDR_TRANSPORT_ETHERNET
static uint8_t *s_s31_parlio_dma_storage;
#endif
static bool s_s31_parlio_running;
static uint32_t s_s31_parlio_data_width;
static uint32_t s_s31_parlio_rate_hz;
static bool s_s31_parlio_swapped_pins;

#endif

typedef struct {
  uint32_t mode;
  uint32_t interval_chunks;
  uint32_t interval_offset_chunks;
  uint32_t interval_duration_chunks;
  uint32_t checks_per_chunk;
  uint32_t pre_chunks;
  uint32_t post_chunks;
  uint32_t max_gain;
  uint32_t state_mask;
  uint32_t threshold;
  uint32_t dc_shift;
  bool use_fsm_match;
  bool use_max_gain_match;
  uint16_t sample_offsets[IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK];
} trigger_plan_t;

_Static_assert(sizeof(capture_config_t) == 172u,
               "Python GUI CaptureConfig must match firmware");
_Static_assert(sizeof(iq_chunk_t) == 4152u,
               "Python GUI IqChunk must match firmware");

static TaskHandle_t s_producer_task_handle;

#if CONFIG_IDF_TARGET_ESP32S31
typedef struct {
  const uint8_t *data;
  uint32_t bytes;
  uint32_t sequence;
} s31_parlio_event_t;

static parlio_rx_unit_handle_t s_s31_parlio_unit;
static parlio_rx_delimiter_handle_t s_s31_parlio_delimiter;
static parlio_rx_delimiter_handle_t s_s31_parlio_delimiter_full;
static parlio_rx_delimiter_handle_t s_s31_parlio_delimiter_half;
static void *s_s31_parlio_internal_dma_reserve;
static s31_parlio_event_t s_s31_parlio_events[S31_PARLIO_EVENT_RING_SIZE];
static volatile uint32_t s_s31_parlio_event_head;
static uint32_t s_s31_parlio_event_tail;
static volatile uint32_t s_s31_parlio_callback_overruns;
static volatile uint32_t s_s31_parlio_discontinuities;
/* Descriptor completions and seven-frame output batches are deliberately
 * independent. Retain the unconsumed suffix of one copied ISR event so no
 * sample is discarded when a completion crosses a batch boundary. */
static s31_parlio_event_t s_s31_parlio_pending_event;
static uint32_t s_s31_parlio_pending_offset;
#if CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
static const int8_t s_s31_diag_gpio[16] = {
    8,  9,  10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23,
};
#else
/* RGMII uses GPIO8..19, MDIO/MDC use GPIO5/6, and Wi-Fi reserves GPIO26..32. */
static const int8_t s_s31_diag_gpio[16] = {
    20, 21, 22, 23, 24, 25, 33, 34,
    36, 37, 38, 39, 40, 42, 43, 44,
};
#endif
#endif

static uint32_t s_isr_prev_sa;    /* ISR: previous store_addr (wrap detect) */
static uint32_t s_isr_wrap_count; /* ISR: number of store_addr wraps */
static volatile uint32_t s_dbg_lo;

static uint32_t s_emit_chunk; /* next absolute chunk index to emit (cursor) */
static bool s_s31_ring_started;
static uint8_t s_chunk_mark[2u * IQ_CHUNKS_PER_BANK];

static volatile uint32_t s_stream_dropped_chunks;
static volatile uint32_t s_source_chunk_index;
static volatile bool s_producer_active;
static volatile bool s_stream_write_active;
static volatile bool s_network_capture_armed;
#if CONFIG_IDF_TARGET_ESP32S31
static volatile uint32_t s_s31_copy_ctrl_diag;
static volatile uint32_t s_s31_copy_mode_before_diag;
static volatile uint32_t s_s31_copy_mode_after_diag;
static volatile uint32_t s_s31_copy_physical_chunk;
static volatile uint32_t s_s31_copy_source_chunk;
static volatile uint64_t s_s31_diag_abs_write;
static volatile uint32_t s_s31_diag_c_hi;
static volatile uint32_t s_s31_diag_emit_chunk;
static volatile uint32_t s_s31_diag_ring_started;
static uint64_t s_s31_abs_write_accum;
static int64_t s_s31_abs_track_us;
static uint32_t s_s31_decim_buf[IQ_CHUNK_SAMPLE_WORDS];
static bool s_s31_stage_valid;
static uint32_t s_s31_stage_source_chunk;
static uint32_t s_s31_stage_prev_end_raw;
static bool s_s31_stage_have_prev_end;
static uint32_t s_s31_micro_gate_advances;
static uint32_t s_s31_micro_gate_max_cycles;
static uint32_t s_s31_micro_gate_batches;
static uint32_t s_s31_cursor_min_backlog;
static uint32_t s_s31_cursor_max_backlog;
static uint32_t s_s31_cursor_batches;
static uint32_t s_s31_cursor_last_writer_cycle;
static uint32_t s_s31_cursor_last_backlog;
static bool s_s31_cursor_have_writer_cycle;
static uint32_t s_s31_cursor_overruns;
static uint32_t s_s31_probe_source_chunk;
static uint32_t s_s31_probe_end_cycle;
static bool s_s31_probe_have_end_cycle;
static uint32_t s_s31_probe_logged_selector = UINT32_MAX;
#endif
static bool s_power_trigger_dc_valid;
static int32_t s_power_trigger_dc_i_q16;
static int32_t s_power_trigger_dc_q_q16;

static portMUX_TYPE s_config_mux = portMUX_INITIALIZER_UNLOCKED;
#if CONFIG_IDF_TARGET_ESP32S31
static portMUX_TYPE s_tcm_dump_gate_mux = portMUX_INITIALIZER_UNLOCKED;
#endif

static capture_config_t s_config = {
    .stream = {.stream_wifi_packets = 0},
    .radio = {.rf_freq_hz = RF_FREQ_DEFAULT_HZ, .frequency_correction_ppb = 0},
    .gain = {.gain_mode = GAIN_MODE_HARDWARE,
             .rx_gain = 32},
    .bandwidth = {.bw_mhz = 20, .second_chan = SECOND_CHAN_NONE},
    .iq_engine = {.adc_decimation = 1,
                  .adc_source_sel = ADC_DUMP_SOURCE_CONTINUOUS_RX_IQ},
    /* interval in CHUNKS. Keep this coprime with the S31 31-slot dump ring so
     * displayed chunks walk slots instead of reusing one physical slot forever.
     */
    .trigger = {.trigger_mode = IQ_TRIGGER_MODE_INTERVAL,
                .trigger_config = {2501u, 0u, 1u}},
    .rx_filter = {.filter_bw_mhz = S31_DEFAULT_FILTER_BW_MHZ,
                  .rx_filter_override = 0,
                  .rx_filter_mode = 16,
                  .rx_filter_dcap = RX_FILTER_NARROW_DCAP},
    .dc_offset = {.automatic = 1},
};
static trigger_plan_t s_trigger_plan;
static capture_config_t s_pending_config;
static trigger_plan_t s_pending_trigger_plan;
static volatile bool s_config_apply_pending;
static volatile bool s_config_apply_enabled;
static volatile bool s_config_apply_in_progress;
static volatile int64_t s_config_apply_started_us;
#if CONFIG_IDF_TARGET_ESP32S31
#endif

/* Source-side IQC8 is disabled on S31: the closed staging view contains the
 * poison sentinel. Ethernet quantizes a proven IQC1 frame in its transport
 * task; high-speed USB retains IQC1 on the wire. */
static inline bool stream_output_int8(void) {
  return false;
}

static void apply_pending_config(void);
static void service_pending_config(void);
static bool config_apply_in_progress(void);
static bool capture_engine_running(const capture_config_t *config);
static trigger_plan_t build_trigger_plan(const capture_config_t *config);
static uint32_t adc_dump_store_addr_raw(void);
#if CONFIG_IDF_TARGET_ESP32S31
static inline bool s31_continuous_real_if_selected(
    const capture_config_t *config) {
#if CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF && CONFIG_ESP_SDR_TRANSPORT_USB && \
    !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  return config->rx_filter.rx_filter_override == 0u;
#else
  (void)config;
  return false;
#endif
}

static inline bool s31_production_native_iq_selected(
    const capture_config_t *config) {
#if CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF && CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  return config->rx_filter.rx_filter_override == 0u;
#else
  (void)config;
  return false;
#endif
}

static capture_config_t
s31_engine_config(const capture_config_t *config) {
  capture_config_t engine = *config;
  if (s31_continuous_real_if_selected(config)) {
    engine.rx_filter.rx_filter_override = S31_GPIO_DIAG_OVERRIDE;
    engine.rx_filter.rx_filter_mode = S31_CONTINUOUS_DIAG_MODE;
  } else if (s31_production_native_iq_selected(config)
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
             || config->rx_filter.rx_filter_override ==
                    S31_PARLIO_HOST_IQ_OVERRIDE
#endif
  ) {
    /* Mode 62 is the legacy SoapyESPSDR request.  It must use the same modem
     * formatter/filter setup as production mode 0; changing only the GPIO
     * route leaves a single tone recognizable but corrupts broadband IQ. */
    engine.rx_filter.rx_filter_override = S31_PARLIO_NATIVE_IQ_OVERRIDE;
    engine.rx_filter.rx_filter_mode = S31_PARLIO_NATIVE_PACKED_IQ;
  }
  return engine;
}

static bool s31_pulse_tcm_before_copy(const capture_config_t *config);
static void s31_pulse_tcm_dump_gate(const capture_config_t *config);
#endif

/* ---- config -> subsystem mappers ---- */

static void init_idf_services(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  } else {
    ESP_ERROR_CHECK(ret);
  }

  ret = esp_netif_init();
  if (ret != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(ret);
  }

  ret = esp_event_loop_create_default();
  if (ret != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(ret);
  }
}

#if CONFIG_IDF_TARGET_ESP32S31 && CONFIG_ESP_PHY_ENABLE_CERT_TEST
static void init_s31_rftest_services(void) {
  esp_wifi_power_domain_on();
  esp_phy_rftest_config(1);
}
#endif

static wifi_rx_config_t
wifi_rx_config_from_capture(const capture_config_t *config) {
  return (wifi_rx_config_t){
      .rf_freq_hz = config->radio.rf_freq_hz,
      .bw_mhz = config->bandwidth.bw_mhz,
      .second_chan = config->bandwidth.second_chan,
      .stream_packets = config->stream.stream_wifi_packets,
  };
}

static modem_config_t
modem_config_from_capture(const capture_config_t *config) {
  uint32_t hardware_rf_hz = config->radio.rf_freq_hz;
#if CONFIG_IDF_TARGET_ESP32S31
  if (s31_continuous_real_if_selected(config)) {
    hardware_rf_hz = hardware_rf_hz > RF_FREQ_MIN_HZ + S31_CONTINUOUS_IF_HZ
                         ? hardware_rf_hz - S31_CONTINUOUS_IF_HZ
                         : RF_FREQ_MIN_HZ;
#if CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  } else if (config->rx_filter.rx_filter_override ==
             S31_PARLIO_HOST_IQ_OVERRIDE) {
    /* The host selects the positive-frequency half of the real ADC stream,
     * translates this Fs/4 low IF to baseband, and decimates by two. */
    uint32_t host_rate = S31_PARLIO_HOST_REAL_RATE_HZ;
#if !CONFIG_ESP_SDR_TRANSPORT_USB
    if ((config->rx_filter.rx_filter_mode & S31_PARLIO_HOST_SYNC_40M) != 0u) {
      host_rate = S31_PARLIO_HOST_SYNC_RATE_HZ;
    }
#endif
    uint32_t host_if_hz = host_rate / 4u;
    hardware_rf_hz = hardware_rf_hz > RF_FREQ_MIN_HZ + host_if_hz
                         ? hardware_rf_hz - host_if_hz
                         : RF_FREQ_MIN_HZ;
#endif
  }
#endif
  return (modem_config_t){
      .rf_freq_hz = hardware_rf_hz,
      .frequency_correction_ppb = config->radio.frequency_correction_ppb,
      .bw_mhz = config->bandwidth.bw_mhz,
      .second_chan = config->bandwidth.second_chan,
      .gain_mode = config->gain.gain_mode,
      .rx_gain = config->gain.rx_gain,
      .expert_gain_word0 = config->gain.expert_gain_word0,
      .expert_gain_word1 = config->gain.expert_gain_word1,
      .expert_gain_word2 = config->gain.expert_gain_word2,
      .filter_bw_mhz = config->rx_filter.filter_bw_mhz,
      .rx_filter_override = config->rx_filter.rx_filter_override,
      .rx_filter_mode = config->rx_filter.rx_filter_mode,
      .rx_filter_dcap = config->rx_filter.rx_filter_dcap,
      .dc_offset_automatic = config->dc_offset.automatic,
  };
}

static inline int32_t sign_extend_10_u32(uint32_t value) {
  value &= 0x3ffu;
  return (value & 0x200u) != 0u ? (int32_t)value - 0x400 : (int32_t)value;
}

static inline uint32_t abs_i32_u32(int32_t value) {
  return (uint32_t)(value < 0 ? -value : value);
}

/* ---- ADC dump engine ---- */

static void IRAM_ATTR engine_own_bank(uint32_t bank) {
#if CONFIG_IDF_TARGET_ESP32S31
  (void)bank;
  /*
   * ESP32-S31 does not expose the C61 HP_SYSTEM_SRAM_USAGE_CONF_REG. Its
   * HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG is a dump gate used by S31 rftest
   * adctrig, not a per-bank mac-dump SRAM ownership selector.
   */
  __asm__ __volatile__("fence" ::: "memory");
  (void)reg32_read_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG);
#else
  (void)bank;
#endif
}

#if CONFIG_IDF_TARGET_ESP32S31
static void s31_poison_dump_window(void) {
  for (uint32_t i = 0u; i < DUMP_BANK_WORDS; ++i) {
    DUMP_BASE[i] = S31_POISON_WORD ^ i;
  }
  __asm__ __volatile__("fence" ::: "memory");
  (void)esp_cache_msync((void *)DUMP_BASE, DUMP_BANK_WORDS * sizeof(uint32_t),
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                            ESP_CACHE_MSYNC_FLAG_INVALIDATE);
}
#endif

static uint32_t adc_decimation_value(const capture_config_t *config) {
  uint32_t decimation = config->iq_engine.adc_decimation;
  if (decimation == 0u) {
    return 1u;
  }
  if (decimation > ADC_DECIMATION_MAX) {
    return ADC_DECIMATION_MAX;
  }
  return decimation;
}

#if CONFIG_IDF_TARGET_ESP32S31
static uint32_t s31_hardware_decimation_factor(
    const capture_config_t *config) {
  uint32_t field =
      (config->iq_engine.adc_source_sel & S31_ADC_SOURCE_HW_DECIM_M) >>
      S31_ADC_SOURCE_HW_DECIM_S;
  /* Differential Pluto-tone measurements identify the useful lower-rate
   * hardware divisors.  All retain the same densely packed IQ word layout. */
  switch (field) {
  case 7u:
    return 10u; /* 8.000 MS/s */
  case 8u:
    return 12u; /* 6.667 MS/s */
  case 9u:
    return 20u; /* 4.000 MS/s */
  case 10u:
    return 24u; /* 3.333 MS/s */
  default:
    return 1u;
  }
}

static uint32_t s31_software_decimation(const capture_config_t *config) {
  if (s31_hardware_decimation_factor(config) != 1u) {
    return 1u;
  }
  return adc_decimation_value(config);
}
#endif

static uint32_t adc_dump_write_sample_cycles(const capture_config_t *config) {
#if CONFIG_IDF_TARGET_ESP32S31
  return 2u * s31_hardware_decimation_factor(config);
#else
  (void)config;
  return 2u;
#endif
}

static uint32_t adc_dump_sample_cycles(const capture_config_t *config) {
#if CONFIG_IDF_TARGET_ESP32S31
  return adc_dump_write_sample_cycles(config) *
         s31_software_decimation(config);
#else
  return 2u * adc_decimation_value(config);
#endif
}

#if CONFIG_IDF_TARGET_ESP32S31
static void s31_enable_data_dump_clocks(const capture_config_t *config) {
  (void)config;
  reg32_set_bits_addr(HP_SYSTEM_TCM_RAM_PWR_CTRL0_REG,
                      HP_SYSTEM_REG_HP_SYSTEM_TCM_CLK_FORCE_ON);
  reg32p_set_bits(&MODEM_SYSCON.CLK_CONF, BIT(31) | BIT(21));
  reg32p_set_bits(&MODEM_SYSCON.CLK_CONF_FORCE_ON, BIT(31));
  reg32p_set_bits(&MODEM_SYSCON.CLK_CONF1, 0x0000e400u);
  reg32p_write_field(&MODEM_SYSCON.CLK_CONF_POWER_ST, 0x0000f000u, 12u, 4u);
  reg32p_write_field(&MODEM_SYSCON.CLK_CONF_POWER_ST, 0x0f000000u, 24u, 4u);
  reg32p_write_field(&MODEM_SYSCON.CLK_CONF_POWER_ST, 0xf0000000u, 28u, 4u);
  reg32p_set_bits(&MODEM_SYSCON.MODEM_RST_CONF, BIT(31));
  reg32p_clear_bits(&MODEM_SYSCON.MODEM_RST_CONF, BIT(31));
}
#endif

#if CONFIG_IDF_TARGET_ESP32S31
static void s31_configure_modem_diag(const capture_config_t *config) {
  uint32_t expert = config->rx_filter.rx_filter_override;
  if (!((expert >= S31_MODEM_DIAG_PROBE_OVERRIDE_FIRST &&
         expert <= S31_MODEM_DIAG_PROBE_OVERRIDE_LAST) ||
        expert == S31_GPIO_DIAG_OVERRIDE ||
        expert == S31_PARLIO_NATIVE_IQ_OVERRIDE ||
        expert == S31_PARLIO_HOST_IQ_OVERRIDE)) {
    reg32_write_addr(HP_SYSTEM_MODEM_DIAG_EN_REG, 0u);
    return;
  }
  uint32_t diagnostic_mode = config->rx_filter.rx_filter_mode;
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  if (expert == S31_PARLIO_HOST_IQ_OVERRIDE) {
    /* SoapyESPSDR versions that predate native I/Q request expert mode 62.
     * Keep them wire-compatible, but feed the new native complex route. */
    expert = S31_PARLIO_NATIVE_IQ_OVERRIDE;
    diagnostic_mode = S31_PARLIO_NATIVE_PACKED_IQ;
  }
#else
  if (expert == S31_PARLIO_HOST_IQ_OVERRIDE) {
    /* Reuse the proven production real-IF route. Only the eight ADC MSBs are
     * sampled below; no per-sample conversion is performed on the MCU. */
    expert = S31_GPIO_DIAG_OVERRIDE;
    diagnostic_mode = S31_CONTINUOUS_DIAG_MODE;
  }
#endif
  uint32_t selector = diagnostic_mode & 31u;
  uint32_t variant = (expert == S31_GPIO_DIAG_OVERRIDE ||
                      expert == S31_PARLIO_NATIVE_IQ_OVERRIDE)
                         ? 0u
                         : (expert - S31_MODEM_DIAG_PROBE_OVERRIDE_FIRST) / 2u;
  uint32_t exchange =
      (variant == 1u || variant == 3u) ? 0x0123u : 0x3210u;
  if (expert == S31_GPIO_DIAG_OVERRIDE &&
      (diagnostic_mode & BIT(23)) != 0u) {
    uint32_t byte0 = (diagnostic_mode >> 15u) & 15u;
    uint32_t byte1 = (diagnostic_mode >> 19u) & 15u;
    uint32_t byte2 = (diagnostic_mode >> 24u) & 15u;
    uint32_t byte3 = (diagnostic_mode >> 28u) & 15u;
    exchange = (byte3 << 12u) | (byte2 << 8u) | (byte1 << 4u) | byte0;
  }
  if (expert == S31_PARLIO_NATIVE_IQ_OVERRIDE) {
    selector = S31_CONTINUOUS_DIAG_MODE & 31u;
    if ((diagnostic_mode & BIT(31)) != 0u) {
      exchange = (diagnostic_mode & BIT(30)) != 0u
                     ? (diagnostic_mode >> 8u) & 0xffffu
                     : 0x3210u;
    } else {
      exchange = 0x32f2u;
    }
  }
  bool gpio_high_half = expert == S31_GPIO_DIAG_OVERRIDE &&
                        (diagnostic_mode & BIT(14)) != 0u;
  uint32_t fix_sel = gpio_high_half
                         ? selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_HIGH_S
                         : selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_LOW_S;
  if (variant == 2u || variant == 3u) {
    fix_sel |= selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_HIGH_S;
  } else if (variant == 4u) {
    fix_sel = selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_HIGH_S;
  }
  bool gpio_dynamic_exchange = expert == S31_GPIO_DIAG_OVERRIDE &&
                               (diagnostic_mode & BIT(13)) != 0u;
  if (variant != 4u && !gpio_high_half && !gpio_dynamic_exchange) {
    fix_sel |= MODEM_WIDGETS_DIAG_BUS_FIX_LOW_EN;
  }
  if (expert == S31_PARLIO_NATIVE_IQ_OVERRIDE) {
    uint32_t high_selector = (diagnostic_mode >> 9u) & 31u;
    if ((diagnostic_mode & BIT(31)) != 0u) {
      /* Exchange-search mode: leave the low half on the byte exchange.  The
       * optional full-low route (bit 30) observes both exchanged bytes; the
       * older mixed route keeps the high half fixed for diagnostic A/Bs. */
      high_selector = (diagnostic_mode & BIT(30)) != 0u
                          ? selector
                          : (diagnostic_mode >> 8u) & 31u;
      fix_sel = high_selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_HIGH_S;
    } else {
      /* Fixed-half census mode. */
      fix_sel = MODEM_WIDGETS_DIAG_BUS_FIX_LOW_EN |
                (selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_LOW_S) |
                (high_selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_HIGH_S);
    }
  }
  reg32_set_bits_addr(HP_SYSTEM_CLK_EN_REG, HP_SYSTEM_REG_CLK_EN);
  reg32_set_bits_addr(HP_SYSTEM_PROBEA_CTRL_REG,
                      HP_SYSTEM_REG_PROBE_GLOBAL_EN);
  reg32_set_bits_addr(MODEM_WIDGETS_CLK_CONF_REG, MODEM_WIDGETS_CLK_EN);
  reg32_write_addr(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG, exchange);
  reg32_write_addr(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG, fix_sel);
  if (expert == S31_GPIO_DIAG_OVERRIDE ||
      expert == S31_PARLIO_NATIVE_IQ_OVERRIDE) {
    uint32_t bb_diag = reg32p_read(&MODEM_WIFI_BB.BB_DIAG0);
    bb_diag &= ~(0xffu | BIT(30));
    bb_diag |= (expert == S31_PARLIO_NATIVE_IQ_OVERRIDE
                    ? ((diagnostic_mode & BIT(31)) != 0u
                           ? ((diagnostic_mode >> 24u) & 0x07u) |
                                 (diagnostic_mode & BIT(3)) |
                                 (((diagnostic_mode >> 4u) & 0x0fu) << 4u)
                           : diagnostic_mode >> 14u)
                    : (diagnostic_mode >> 5u)) &
               0xffu;
    if ((expert == S31_GPIO_DIAG_OVERRIDE &&
         (diagnostic_mode & BIT(13)) != 0u) ||
        (expert == S31_PARLIO_NATIVE_IQ_OVERRIDE &&
         (diagnostic_mode & BIT(31)) != 0u)) {
      /* BB_DIAG0[30] is the undocumented enable for the widgets byte
       * exchange.  Without it, MODEM_DIAG_EXCHANGE reads back correctly but
       * the dynamic low half remains on its reset/default sources. */
      bb_diag |= BIT(30);
    }
    reg32p_write(&MODEM_WIFI_BB.BB_DIAG0, bb_diag);
  }
  uint32_t diag_enable =
      gpio_high_half ? 0xffff0000u : 0x0000ffffu;
  if (expert == S31_PARLIO_NATIVE_IQ_OVERRIDE) {
    /* The S31 substitution gate is slice-wide in practice: enabling only the
     * eight routed bits leaves LCD_DATA on the matrix on rev-0 silicon. */
    diag_enable = (diagnostic_mode & BIT(30)) != 0u
                      ? 0x0000ffffu
                      : 0xffffffffu;
  }
  reg32_write_addr(HP_SYSTEM_MODEM_DIAG_EN_REG, diag_enable);
}
#endif

static void adctrig_prepare(uint32_t sample_count,
                            const capture_config_t *config) {
  uint32_t source_sel = config->iq_engine.adc_source_sel;
#if CONFIG_IDF_TARGET_ESP32S31
  source_sel &= S31_ADC_DUMP_SOURCE_SEL_MASK;
#endif
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_clear_bits_addr(PHY_MODEM_BASE_ADDR + 0x08ccu, 0x00780000u);
  reg32_clear_bits_addr(PHY_MODEM_BASE_ADDR + 0x70b8u, 0x7u);
  reg32_set_bits_addr(PHY_MODEM_BASE_ADDR + 0x70b8u, BIT(0));
  reg32_set_bits_addr(MODEM_WIFI_S31_ADCTRIG_SETUP_REG,
                      MODEM_WIFI_S31_ADCTRIG_SETUP_BIT);
  reg32_write_addr(MODEM_WIFI_S31_ADCTRIG_CFG_REG, UINT32_MAX);
#endif
  reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                    MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
  uint32_t hardware_decimation_field =
      (config->iq_engine.adc_source_sel & S31_ADC_SOURCE_HW_DECIM_M) >>
      S31_ADC_SOURCE_HW_DECIM_S;
#if CONFIG_IDF_TARGET_ESP32S31
  /* This register-only reset must occur while the S31 dump fabric is open;
   * otherwise the formatter/source setup does not latch into the active dump
   * path. PHY blob and I2C calls remain outside this transaction. */
  s31_enable_data_dump_clocks(config);
#else
  MODEM_SYSCON.CLK_CONF = 0xffffffffu;
#endif
  /* S31 clock bring-up pulses the modem reset, which clears ADC_DUMP_MODE.
   * Program the latched rate field only after that reset has completed. */
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_MODE,
                     MODEM_WIFI_DUMP_MODE_DECIM_M, MODEM_WIFI_DUMP_MODE_DECIM_S,
                     hardware_decimation_field);
  reg32p_set_bits(&MODEM_WIFI_FE_CTRL.CLK_ENABLE, 0x00000004u);

  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                     MODEM_WIFI_DUMP_CTRL_SAMPLE_COUNT_M,
                     MODEM_WIFI_DUMP_CTRL_SAMPLE_COUNT_S, sample_count);
  reg32p_set_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                  MODEM_WIFI_DUMP_CTRL_DONE_BIT);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_DONE_BIT);

  uint32_t byte_sel[4] = {24u, 25u, 26u, 27u};
  const bool vendor_mode11_pack =
      config->rx_filter.rx_filter_override == 14u ||
      config->rx_filter.rx_filter_override == 15u;
  if (vendor_mode11_pack) {
    /* Exact S31 librftest high-level dump mode 11 packing.  Its source-map
     * branch separately selects raw ADC_DUMP_MODE source 11. */
    byte_sel[0] = 20u;
    byte_sel[1] = 21u;
    byte_sel[2] = 22u;
    byte_sel[3] = 23u;
  } else if (config->rx_filter.rx_filter_override == 30u ||
             config->rx_filter.rx_filter_override == 31u ||
             config->rx_filter.rx_filter_override == 32u ||
             config->rx_filter.rx_filter_override == 33u) {
    /* Stable candidate pair from the full BT-loopback census.  Keep filter
     * control in rx_filter_mode while exposing selectors 8/9 directly. */
    byte_sel[0] = 8u;
    byte_sel[1] = 9u;
    byte_sel[2] = 8u;
    byte_sel[3] = 9u;
  } else if ((config->rx_filter.rx_filter_override >= 2u &&
              config->rx_filter.rx_filter_override < 8u) ||
             config->rx_filter.rx_filter_override == 28u ||
             config->rx_filter.rx_filter_override == 29u) {
    uint32_t pair = config->rx_filter.rx_filter_mode;
    if (pair < 32u) {
      /* Diagnostic pair scan: expose every adjacent pair in the 64-byte
       * adctrig debug bus through word bytes 0/1.  Repeat it in bytes 2/3 so
       * both the direct-byte and raw-word readers see the same candidate. */
      byte_sel[0] = pair * 2u;
      byte_sel[1] = pair * 2u + 1u;
      byte_sel[2] = byte_sel[0];
      byte_sel[3] = byte_sel[1];
    } else {
      /* Exact S31 librftest adctrig mode-12 Bluetooth packing. */
      byte_sel[0] = 20u;
      byte_sel[1] = 21u;
      byte_sel[2] = 8u;
      byte_sel[3] = 11u;
    }
  } else if (config->rx_filter.rx_filter_override == 34u ||
             config->rx_filter.rx_filter_override == 35u) {
    /* The real BTRX-path census found activity only on selectors
     * 8,9,20,21,24,25.  Adjacent pairs have poor image rejection, so test
     * every ordered cross-combination instead of assuming I and Q are
     * adjacent on the debug bus. */
    static const uint8_t active_sel[6] = {8u, 9u, 20u, 21u, 24u, 25u};
    uint32_t pair = config->rx_filter.rx_filter_mode;
    byte_sel[0] = active_sel[(pair / 6u) % 6u];
    byte_sel[1] = active_sel[pair % 6u];
    byte_sel[2] = byte_sel[0];
    byte_sel[3] = byte_sel[1];
  } else if (config->rx_filter.rx_filter_override == 36u ||
             config->rx_filter.rx_filter_override == 37u) {
    /* Normal Wi-Fi antenna route, exhaustive adjacent debug-byte pair. */
    uint32_t pair = config->rx_filter.rx_filter_mode & 31u;
    byte_sel[0] = pair * 2u;
    byte_sel[1] = pair * 2u + 1u;
    byte_sel[2] = byte_sel[0];
    byte_sel[3] = byte_sel[1];
  }
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                     MODEM_WIFI_DUMP_CFG_BYTE3_SEL_M,
                     MODEM_WIFI_DUMP_CFG_BYTE3_SEL_S, byte_sel[3]);
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                     MODEM_WIFI_DUMP_CFG_BYTE2_SEL_M,
                     MODEM_WIFI_DUMP_CFG_BYTE2_SEL_S, byte_sel[2]);
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                     MODEM_WIFI_DUMP_CFG_BYTE1_SEL_M,
                     MODEM_WIFI_DUMP_CFG_BYTE1_SEL_S, byte_sel[1]);
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                     MODEM_WIFI_DUMP_CFG_BYTE0_SEL_M,
                     MODEM_WIFI_DUMP_CFG_BYTE0_SEL_S, byte_sel[0]);
  if (config->rx_filter.rx_filter_override ==
          S31_SEAMLESS_MICRO_GATE_OVERRIDE ||
      config->rx_filter.rx_filter_override == S31_GPIO_DIAG_OVERRIDE ||
      config->rx_filter.rx_filter_override ==
          S31_PARLIO_NATIVE_IQ_OVERRIDE ||
      config->rx_filter.rx_filter_override == S31_PARLIO_HOST_IQ_OVERRIDE) {
    reg32p_set_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                    MODEM_WIFI_DUMP_CFG_AGC_DBG_EN);
  } else if (config->rx_filter.rx_filter_override >= 2u &&
      (config->rx_filter.rx_filter_override & 1u) == 0u) {
    reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                      MODEM_WIFI_DUMP_CFG_AGC_DBG_EN);
  } else {
    /* The production 24/25/26/27 IQ selectors require the debug formatter.
     * Override parity is an expert A/B control only; never let calibrated
     * override 0 disable the normal antenna producer. */
    reg32p_set_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                    MODEM_WIFI_DUMP_CFG_AGC_DBG_EN);
  }

  /* engine owns bank 0 to start; CPU owns bank 1. */
  engine_own_bank(0u);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_RESERVED_MODE_M);
  /* Continuous mode maps the writer into the reserved streaming aperture.
   * The vendor's finite Bluetooth test route clears this bit and writes from
   * TCM offset zero, which overlaps the application in an IDF firmware. */
  reg32p_set_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                  MODEM_WIFI_DUMP_CTRL_CONTINUOUS_TRIGGER_GATE_BIT);

#if !CONFIG_IDF_TARGET_ESP32S31
  reg32p_set_bits(&MODEM_WIFI_FE_TRIGGER.FE_DUMP_TRIGGER_CTRL,
                  MODEM_WIFI_FE_DUMP_TRIGGER_CTRL_ENABLE);
  reg32p_write_field(&MODEM_WIFI_FE_TRIGGER.FE_DUMP_TRIGGER_CTRL,
                     MODEM_WIFI_FE_DUMP_TRIGGER_CTRL_ARG_M, 0u, 0u);
#endif

#if !CONFIG_IDF_TARGET_ESP32S31
  reg32p_clear_bits(&MODEM_WIFI_DUMP_MISC.DUMP_SOURCE_GATE,
                    MODEM_WIFI_DUMP_SOURCE_GATE_EN);
#endif
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_MODE,
                     MODEM_WIFI_DUMP_MODE_SOURCE_SEL_M,
                     MODEM_WIFI_DUMP_MODE_SOURCE_SEL_S, source_sel);

  /* The S31 exposes the modem's live diagnostic bus independently of TCM. */
  s31_configure_modem_diag(config);

  if (config->rx_filter.rx_filter_override == S31_LP_CORE_PROBE_OVERRIDE) {
    /* Route all four bytes of one LP-core internal signal group to the LP
     * probe output.  Group 15 is documented as the target identifier; the
     * remaining groups are swept on real hardware to find the load/GPR bus. */
    uint32_t group = config->rx_filter.rx_filter_mode & 15u;
    uint32_t mod_sel = group * 0x1111u;
    uint32_t ctrl = mod_sel |
                    (3u << LP_SYSTEM_REG_PROBE_A_TOP_SEL_S) |
                    (1u << LP_SYSTEM_REG_PROBE_H_SEL_S) |
                    LP_SYSTEM_REG_PROBE_GLOBAL_EN;
    reg32_write_addr(LP_SYSTEM_REG_LP_PROBEB_CTRL_REG, 0u);
    reg32_write_addr(LP_SYSTEM_REG_LP_PROBEA_CTRL_REG, ctrl);
  } else {
    reg32_write_addr(LP_SYSTEM_REG_LP_PROBEA_CTRL_REG, 0u);
    reg32_write_addr(LP_SYSTEM_REG_LP_PROBEB_CTRL_REG, 0u);
  }

  if (config->rx_filter.rx_filter_override == S31_HP_TCM_PROBE_OVERRIDE) {
    /* Observe the TCM controller itself, rather than attempting a memory
     * access through a CPU/DMA master.  If its write-data group is exposed,
     * this remains live while the modem owns the aperture. */
    uint32_t mode = config->rx_filter.rx_filter_mode;
    uint32_t group = mode & 15u;
    uint32_t mod_sel = group * 0x1111u;
    if ((mode & BIT(23)) != 0u) {
      /* Expert census mode: reuse the modem-probe script's four-nibble
       * encoding so each byte of probe channel A can select an independent
       * TCM signal group.  Only byte 0/1 reach GPIO in the lower-16 route,
       * but retaining all four makes the register readback unambiguous. */
      mod_sel = ((mode >> 15u) & 0x0fu) |
                (((mode >> 19u) & 0x0fu) << 4u) |
                (((mode >> 24u) & 0x0fu) << 8u) |
                (((mode >> 28u) & 0x0fu) << 12u);
    }
    uint32_t ctrl = mod_sel |
                    (3u << HP_SYSTEM_REG_PROBE_A_TOP_SEL_S) |
                    (1u << HP_SYSTEM_REG_PROBE_H_SEL_S) |
                    HP_SYSTEM_REG_PROBE_GLOBAL_EN;
    reg32_write_addr(HP_SYSTEM_PROBEB_CTRL_REG, 0u);
    reg32_write_addr(HP_SYSTEM_PROBEA_CTRL_REG, ctrl);
  }
}

static inline uint32_t IRAM_ATTR adc_dump_store_addr_raw(void) {
  return (MODEM_WIFI_DUMP.ADC_DUMP_MODE & MODEM_WIFI_DUMP_MODE_STORE_ADDR_M) >>
         MODEM_WIFI_DUMP_MODE_STORE_ADDR_S;
}

static inline uint32_t IRAM_ATTR adc_dump_store_addr_from_raw(uint32_t addr) {
  return addr;
}

static inline uint32_t IRAM_ATTR adc_dump_store_addr(void) {
  return adc_dump_store_addr_from_raw(adc_dump_store_addr_raw());
}

static inline uint32_t IRAM_ATTR dump_ring_mod(uint32_t value) {
  return value % DUMP_BANK_WORDS;
}

static inline uint32_t IRAM_ATTR dump_ring_delta(uint32_t cur, uint32_t prev) {
  return (cur + DUMP_BANK_WORDS - prev) % DUMP_BANK_WORDS;
}

static bool IRAM_ATTR s31_parlio_partial_receive(
    parlio_rx_unit_handle_t unit, const parlio_rx_event_data_t *event,
    void *user_data) {
  (void)unit;
  (void)user_data;
  uint32_t sequence = s_s31_parlio_event_head;
  s31_parlio_event_t *slot =
      &s_s31_parlio_events[sequence % S31_PARLIO_EVENT_RING_SIZE];
  uint32_t bytes = event->recv_bytes;
  if (bytes <= S31_PARLIO_EVENT_MAX_BYTES) {
    /* Publish the completed circular GDMA node itself. The producer retains
     * ownership until it has consumed every byte, avoiding a full
     * PSRAM-to-PSRAM copy in this ISR. */
    slot->data = event->data;
    slot->bytes = bytes;
  } else {
    slot->data = NULL;
    slot->bytes = 0u;
    ++s_s31_parlio_callback_overruns;
  }
  slot->sequence = sequence;
  __atomic_store_n(&s_s31_parlio_event_head, sequence + 1u,
                   __ATOMIC_RELEASE);
  BaseType_t task_woken = pdFALSE;
  if (s_producer_task_handle != NULL) {
    vTaskNotifyGiveFromISR(s_producer_task_handle, &task_woken);
  }
  return task_woken == pdTRUE;
}

static uint32_t s31_parlio_diag_rate_hz(const capture_config_t *config) {
  uint32_t divider = adc_decimation_value(config);
  return S31_PARLIO_DIAG_RATE_HZ / divider;
}

static bool s31_parlio_host_iq_selected(const capture_config_t *config) {
#if CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  return config->rx_filter.rx_filter_override ==
         S31_PARLIO_HOST_IQ_OVERRIDE;
#else
  (void)config;
  return false;
#endif
}

static bool s31_parlio_native_packed_iq_selected(
    const capture_config_t *config) {
  return s31_production_native_iq_selected(config) ||
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
         config->rx_filter.rx_filter_override ==
             S31_PARLIO_HOST_IQ_OVERRIDE ||
#endif
         (config->rx_filter.rx_filter_override ==
              S31_PARLIO_NATIVE_IQ_OVERRIDE &&
          (config->rx_filter.rx_filter_mode &
           S31_PARLIO_NATIVE_PACKED_IQ) != 0u &&
          (config->rx_filter.rx_filter_mode & BIT(31)) == 0u);
}

static bool s31_parlio_usb_iq4_selected(const capture_config_t *config) {
#if CONFIG_ESP_SDR_TRANSPORT_USB
  return s31_production_native_iq_selected(config) &&
         iq_network_stream_owner() == IQ_STREAM_OWNER_USB &&
         iq_usb_stream_format() == IQ_USB_FORMAT_INT4;
#else
  (void)config;
  return false;
#endif
}

static bool s31_parlio_host_real_selected(const capture_config_t *config) {
  return s31_parlio_host_iq_selected(config) ||
         s31_continuous_real_if_selected(config);
}

static uint32_t s31_parlio_capture_rate_hz(const capture_config_t *config) {
  if (s31_continuous_real_if_selected(config)) {
    return S31_CONTINUOUS_INPUT_RATE_HZ;
  }
  if (!s31_parlio_host_real_selected(config)) {
    return s31_parlio_diag_rate_hz(config);
  }
#if !CONFIG_ESP_SDR_TRANSPORT_USB
  if ((config->rx_filter.rx_filter_mode & S31_PARLIO_HOST_SYNC_40M) != 0u) {
    return S31_PARLIO_HOST_SYNC_RATE_HZ;
  }
#endif
  return S31_PARLIO_HOST_REAL_RATE_HZ;
}

static uint32_t s31_gpio_diag_output_signal(const capture_config_t *config,
                                            uint32_t bit) {
  /* ESP32-S31 exposes the two halves of modem_diag[] through different GPIO
   * matrix sources.  HP_SYSTEM_MODEM_DIAG_EN[15:0] substitutes the low half
   * onto HP_PROBE_TOP_OUT[15:0], while [31:16] substitutes the high half onto
   * LCD_DATA_OUT[15:0] (see hp_system_reg.h). */
  bool high_half = config->rx_filter.rx_filter_override ==
                       S31_GPIO_DIAG_OVERRIDE &&
                   (config->rx_filter.rx_filter_mode & BIT(14)) != 0u;
  if (high_half) {
    return LCD_DATA_OUT_PAD_OUT0_IDX + bit;
  }
  if (s31_parlio_usb_iq4_selected(config)) {
    /* Capture one complex sample per byte without a CPU pack pass. The low
     * nibble is signed I[9:6] from modem_diag[19:16], and the high nibble is
     * signed Q[9:6] from modem_diag[9:6]. */
    if (bit < 4u) {
      return LCD_DATA_OUT_PAD_OUT0_IDX + bit;
    }
    uint32_t source_bit = bit + 2u;
    return source_bit < 8u ? HP_PROBE_TOP_OUT0_IDX + source_bit
                           : HP_PROBE_TOP_OUT8_IDX + source_bit - 8u;
  }
  if (config->rx_filter.rx_filter_override ==
          S31_PARLIO_NATIVE_IQ_OVERRIDE ||
      s31_parlio_native_packed_iq_selected(config)) {
    uint32_t source_bit;
    if (s31_parlio_native_packed_iq_selected(config)) {
      /* Like the TCM dump word, the live diagnostic word carries signed Q in
       * modem_diag[9:0] and signed I in modem_diag[19:10]. Keep their top
       * eight bits and splice I across the two physical substitution groups.
       * The PARLIO pin map below exchanges the two complete physical byte
       * groups, so the captured word is native signed I followed by native
       * signed Q without CPU-side sample conversion:
       *
       *   output[7:0]  = modem_diag[19:12]
       *   output[15:8] = modem_diag[9:2]
       */
      if (bit < 8u) {
        source_bit = bit + 2u;
        return source_bit < 8u ? HP_PROBE_TOP_OUT0_IDX + source_bit
                               : HP_PROBE_TOP_OUT8_IDX + source_bit - 8u;
      }
      if (bit < 12u) {
        return HP_PROBE_TOP_OUT8_IDX + bit - 4u;
      }
      return LCD_DATA_OUT_PAD_OUT0_IDX + bit - 12u;
    }
    if ((config->rx_filter.rx_filter_mode & BIT(30)) != 0u) {
      /* Full-low mode carries two complete exchanged bytes.  Do not apply
       * either of the mixed-route bit-window shifts, because even a one-bit
       * shift would splice the two signed samples together. */
      return bit < 8u ? HP_PROBE_TOP_OUT0_IDX + bit
                      : HP_PROBE_TOP_OUT8_IDX + bit - 8u;
    }
    if (bit < 8u) {
      source_bit = (config->rx_filter.rx_filter_mode & 7u) + bit;
      return source_bit < 8u ? HP_PROBE_TOP_OUT0_IDX + source_bit
                             : HP_PROBE_TOP_OUT8_IDX + source_bit - 8u;
    }
    uint32_t high_shift =
        (config->rx_filter.rx_filter_mode & BIT(31)) != 0u
            ? (config->rx_filter.rx_filter_mode >> 27u) & 7u
            : (config->rx_filter.rx_filter_mode >> 4u) & 7u;
    source_bit = high_shift + bit - 8u;
    return LCD_DATA_OUT_PAD_OUT0_IDX + source_bit;
  }
  if (s31_parlio_host_real_selected(config)) {
    /* modem_diag[9:2] is the signed ADC word with its two least-significant
     * bits discarded. PARLIO packs these lanes directly. */
    return bit < 6u ? HP_PROBE_TOP_OUT0_IDX + bit + 2u
                    : HP_PROBE_TOP_OUT8_IDX + bit - 6u;
  }
  return bit < 8u ? HP_PROBE_TOP_OUT0_IDX + bit
                  : HP_PROBE_TOP_OUT8_IDX + bit - 8u;
}

/* IDF exposes the PARLIO sample clock only as an immutable creation
 * property. Rebuilding the unit for every ordinary sample-rate change is not
 * viable in the combined USB/Ethernet image: the driver gets its descriptors
 * and ISR objects from the small internal heap, which becomes fragmented once
 * both network stacks have run. The S31 divider is independent of the unit's
 * DMA geometry, so retime it while the receiver is disabled and retain every
 * allocation. */
static bool s31_gpio_diag_retime(uint32_t capture_rate_hz) {
  if (s_s31_parlio_unit == NULL || capture_rate_hz == 0u) {
    return false;
  }
  s31_gpio_diag_stop();

  uint32_t source_rate_hz = 0u;
  if (esp_clk_tree_src_get_freq_hz(PARLIO_CLK_SRC_DEFAULT,
                                   ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                   &source_rate_hz) != ESP_OK ||
      source_rate_hz == 0u) {
    return false;
  }
  hal_utils_clk_info_t clock_info = {
      .src_freq_hz = source_rate_hz,
      .exp_freq_hz = capture_rate_hz,
      .max_integ = PARLIO_LL_RX_MAX_CLK_INT_DIV,
      .min_integ = 1u,
      .max_fract = PARLIO_LL_RX_MAX_CLK_FRACT_DIV,
  };
  hal_utils_clk_div_t divider = {.integer = 1u};
  uint32_t actual_rate_hz =
      hal_utils_calc_clk_div_frac_accurate(&clock_info, &divider);
  if (actual_rate_hz == 0u) {
    return false;
  }
  parlio_ll_rx_set_clock_div(&PARL_IO, &divider);
  parlio_ll_rx_update_config(&PARL_IO);
  s_s31_parlio_rate_hz = capture_rate_hz;
  ESP_LOGI("iq_capture", "retimed persistent PARLIO RX clock to %" PRIu32
                         " Hz (actual %" PRIu32 " Hz)",
           capture_rate_hz, actual_rate_hz);
  return true;
}

static void s31_gpio_diag_prepare(const capture_config_t *config) {
  uint32_t expert = config->rx_filter.rx_filter_override;
  if (expert != S31_GPIO_DIAG_OVERRIDE &&
      expert != S31_PARLIO_NATIVE_IQ_OVERRIDE &&
      expert != S31_PARLIO_HOST_IQ_OVERRIDE &&
      expert != S31_HP_TCM_PROBE_OVERRIDE &&
      !s31_continuous_real_if_selected(config) &&
      !s31_parlio_native_packed_iq_selected(config)) {
    return;
  }
  uint32_t data_width =
      (s31_parlio_host_real_selected(config) ||
       s31_parlio_usb_iq4_selected(config))
          ? 8u
          : 16u;
  uint32_t capture_rate_hz = s31_continuous_real_if_selected(config)
                                 ? S31_CONTINUOUS_INPUT_RATE_HZ
                                 : s31_parlio_capture_rate_hz(config);
  bool swapped_pins = s31_parlio_native_packed_iq_selected(config) &&
                      !s31_parlio_usb_iq4_selected(config);
  if (s_s31_parlio_unit != NULL &&
      s_s31_parlio_rate_hz != capture_rate_hz &&
      !s31_gpio_diag_retime(capture_rate_hz)) {
    s31_gpio_diag_release();
  }
  if (s_s31_parlio_unit != NULL) {
    s31_gpio_diag_stop();
    bool geometry_changed = s_s31_parlio_data_width != data_width ||
                            s_s31_parlio_swapped_pins != swapped_pins;
    if (geometry_changed) {
      /* rx_bus_wid_sel is shadowed into the active packing datapath. Merely
       * writing it while stopped works before the first transaction, but an
       * active 16-lane transaction followed by an 8-lane transaction retained
       * the old packing cadence. A core-clock reset only moved the error from
       * 4 to 8 MSa/s. Mirror the driver's fake-EOF recovery instead: reset the
       * complete peripheral register domain while capture and GDMA are idle,
       * restore its retained configuration, then apply the new geometry. No
       * driver allocation or software object is disturbed.
       */
      parl_io_dev_t saved_regs = PARL_IO;
      parlio_ll_reset_register(0);
      memcpy((void *)&PARL_IO, &saved_regs, sizeof(saved_regs));
      parlio_ll_rx_update_config(&PARL_IO);
    }
    for (uint32_t bit = 0u; bit < 16u; ++bit) {
      if (s_s31_diag_gpio[bit] < 0) {
        continue;
      }
      uint32_t signal = s31_gpio_diag_output_signal(config, bit);
      esp_rom_gpio_connect_out_signal(s_s31_diag_gpio[bit], signal, false,
                                      false);
    }
    /* Data width is only immutable in the public driver API. The S31
     * hardware can update it while disabled, and the soft-delimiter driver
     * does not use cfg.data_width when receiving. Keep one persistent 16-lane
     * unit and reconnect its matrix inputs instead of reallocating GDMA and
     * interrupt objects from the fragmented post-network internal heap. */
    for (uint32_t bit = 0u; bit < 16u; ++bit) {
      uint32_t pin_index = swapped_pins ? bit ^ 8u : bit;
      if (s_s31_diag_gpio[pin_index] >= 0) {
        esp_rom_gpio_connect_in_signal(
            s_s31_diag_gpio[pin_index],
            PARLIO_RX_DATA0_PAD_IN_IDX + bit, false);
      }
    }
    parlio_ll_rx_set_bus_width(&PARL_IO, data_width);
    parlio_ll_rx_update_config(&PARL_IO);
    s_s31_parlio_delimiter = s31_parlio_usb_iq4_selected(config)
                                 ? s_s31_parlio_delimiter_half
                                 : s_s31_parlio_delimiter_full;
    s_s31_parlio_data_width = data_width;
    s_s31_parlio_swapped_pins = swapped_pins;
    return;
  }
  uint64_t pin_mask = 0u;
  for (uint32_t bit = 0u; bit < 16u; ++bit) {
    if (s_s31_diag_gpio[bit] >= 0) {
      pin_mask |= 1ull << s_s31_diag_gpio[bit];
    }
  }
  uint64_t reserved_mask = 0u;
  for (uint32_t pin = 0u; pin < SOC_GPIO_PIN_COUNT; ++pin) {
    if (esp_gpio_is_reserved(1ull << pin)) {
      reserved_mask |= 1ull << pin;
    }
  }
  if ((reserved_mask & pin_mask) != 0u) {
    return;
  }
  gpio_config_t gpio_cfg = {
      .pin_bit_mask = pin_mask,
      .mode = GPIO_MODE_INPUT_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
  for (uint32_t bit = 0u; bit < data_width; ++bit) {
    if (s_s31_diag_gpio[bit] < 0) {
      continue;
    }
    uint32_t signal = s31_gpio_diag_output_signal(config, bit);
    esp_rom_gpio_connect_out_signal(s_s31_diag_gpio[bit], signal, false,
                                    false);
  }

  if (s_s31_parlio_unit != NULL) {
    return;
  }
  /* Keep the high-rate circular acquisition ring off external PSRAM in the
   * Ethernet build. At 40 MB/s, simultaneous PARLIO writes and GMAC reads of
   * PSRAM can trip the S31 GDMA fabric; completed internal nodes are copied to
   * the much deeper PSRAM stream ring by the producer instead. */
#if CONFIG_ESP_SDR_TRANSPORT_USB && CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  /* Both high-speed transports leave too little contiguous internal RAM for
   * this ring. Keep one early, stable PSRAM allocation across capture restarts:
   * moving the live GDMA target around the external-memory heap produced
   * boot- and restart-dependent loss at 16 MSa/s. */
  s_s31_parlio_dma_buffer = s_s31_parlio_dma_storage;
#elif CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  uint32_t parlio_caps =
      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
#else
  uint32_t parlio_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
#endif
#if !(CONFIG_ESP_SDR_TRANSPORT_USB && CONFIG_ESP_SDR_TRANSPORT_ETHERNET)
  s_s31_parlio_dma_buffer = heap_caps_aligned_calloc(
      64u, 1u, S31_PARLIO_DMA_BYTES, parlio_caps);
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  if (s_s31_parlio_dma_buffer == NULL) {
    ESP_LOGW("iq_capture",
             "internal PARLIO DMA ring unavailable; falling back to PSRAM");
    s_s31_parlio_dma_buffer = heap_caps_aligned_calloc(
        64u, 1u, S31_PARLIO_DMA_BYTES,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  }
#endif
#endif
  ESP_ERROR_CHECK(s_s31_parlio_dma_buffer != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  parlio_rx_unit_config_t rx_config = {
      .trans_queue_depth = 1u,
      .max_recv_size = S31_PARLIO_DMA_BYTES,
      .dma_burst_size = 64u,
      /* Allocate and connect the maximum geometry once. Runtime selection
       * below narrows the disabled hardware to eight lanes for USB IQC4. */
      .data_width = 16u,
      .clk_src = PARLIO_CLK_SRC_DEFAULT,
      .ext_clk_freq_hz = 0u,
      .exp_clk_freq_hz = capture_rate_hz,
      .clk_in_gpio_num = GPIO_NUM_NC,
      .clk_out_gpio_num = GPIO_NUM_NC,
      .valid_gpio_num = GPIO_NUM_NC,
      .flags = {
          .clk_gate_en = false,
      },
  };
  for (uint32_t bit = 0u; bit < 16u; ++bit) {
    rx_config.data_gpio_nums[bit] = (gpio_num_t)s_s31_diag_gpio[bit];
  }
  /* Release the boot-time contiguous DMA reservation immediately before the
   * driver's descriptor allocation.  Unlike constructing the PARLIO unit at
   * boot, this preserves the first host-requested sample-rate geometry. */
  heap_caps_free(s_s31_parlio_internal_dma_reserve);
  s_s31_parlio_internal_dma_reserve = NULL;
  esp_err_t error = parlio_new_rx_unit(&rx_config, &s_s31_parlio_unit);
  if (error != ESP_OK) {
    ESP_LOGE("iq_capture", "PARLIO RX allocation failed: %s",
             esp_err_to_name(error));
    return;
  }
  s_s31_parlio_data_width = 16u;
  s_s31_parlio_rate_hz = capture_rate_hz;
  s_s31_parlio_swapped_pins = false;
  parlio_rx_soft_delimiter_config_t delimiter_config = {
      .sample_edge = PARLIO_SAMPLE_EDGE_POS,
      .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
      .eof_data_len = S31_PARLIO_BATCH_BYTES,
      .timeout_ticks = 0u,
  };
  error = parlio_new_rx_soft_delimiter(
      &delimiter_config, &s_s31_parlio_delimiter_full);
  if (error != ESP_OK) {
    ESP_LOGE("iq_capture", "PARLIO delimiter allocation failed: %s",
             esp_err_to_name(error));
    s31_gpio_diag_release();
    return;
  }
  delimiter_config.eof_data_len = S31_PARLIO_BATCH_BYTES / 2u;
  error = parlio_new_rx_soft_delimiter(
      &delimiter_config, &s_s31_parlio_delimiter_half);
  if (error != ESP_OK) {
    ESP_LOGE("iq_capture", "PARLIO half delimiter allocation failed: %s",
             esp_err_to_name(error));
    s31_gpio_diag_release();
    return;
  }
  s_s31_parlio_delimiter = s_s31_parlio_delimiter_full;
  parlio_rx_event_callbacks_t callbacks = {
      .on_partial_receive = s31_parlio_partial_receive,
  };
  error = parlio_rx_unit_register_event_callbacks(
      s_s31_parlio_unit, &callbacks, NULL);
  if (error != ESP_OK) {
    ESP_LOGE("iq_capture", "PARLIO callback setup failed: %s",
             esp_err_to_name(error));
    s31_gpio_diag_release();
    return;
  }
  /* Apply the requested lane order, width and delimiter to the retained
   * maximum-width unit before its first receive transaction. */
  s31_gpio_diag_prepare(config);
}

static void s31_gpio_diag_start(const capture_config_t *config) {
  uint32_t expert = config->rx_filter.rx_filter_override;
  if ((expert != S31_GPIO_DIAG_OVERRIDE &&
       expert != S31_PARLIO_NATIVE_IQ_OVERRIDE &&
       expert != S31_PARLIO_HOST_IQ_OVERRIDE &&
       expert != S31_HP_TCM_PROBE_OVERRIDE &&
       !s31_continuous_real_if_selected(config) &&
       !s31_parlio_native_packed_iq_selected(config)) ||
      s_s31_parlio_unit == NULL || s_s31_parlio_running) {
    return;
  }
  s_s31_parlio_event_head = 0u;
  s_s31_parlio_event_tail = 0u;
  s_s31_parlio_callback_overruns = 0u;
  s_s31_parlio_discontinuities = 0u;
  s_s31_parlio_pending_event = (s31_parlio_event_t){0};
  s_s31_parlio_pending_offset = 0u;
  ESP_ERROR_CHECK(parlio_rx_unit_enable(s_s31_parlio_unit, true));
  /* A width update issued while the retained unit's peripheral clock is
   * disabled is not reliably latched on S31. Reassert it after enable but
   * before mounting the DMA transaction; no samples can be in flight yet. */
  parlio_ll_rx_set_bus_width(&PARL_IO, s_s31_parlio_data_width);
  parlio_ll_rx_update_config(&PARL_IO);
  ESP_ERROR_CHECK(parlio_rx_soft_delimiter_start_stop(
      s_s31_parlio_unit, s_s31_parlio_delimiter, true));
  parlio_receive_config_t receive_config = {
      .delimiter = s_s31_parlio_delimiter,
      .flags.partial_rx_en = true,
  };
  ESP_ERROR_CHECK(parlio_rx_unit_receive(
      s_s31_parlio_unit, s_s31_parlio_dma_buffer, S31_PARLIO_DMA_BYTES,
      &receive_config));
  s_s31_parlio_running = true;
}

static void s31_gpio_diag_stop(void) {
  if (!s_s31_parlio_running) {
    return;
  }
  (void)parlio_rx_soft_delimiter_start_stop(
      s_s31_parlio_unit, s_s31_parlio_delimiter, false);
  ESP_ERROR_CHECK(parlio_rx_unit_disable(s_s31_parlio_unit));
  s_s31_parlio_running = false;
}

/* Width and clock are immutable properties of a PARLIO unit. Full receiver
 * reconfiguration must destroy it, otherwise a later 32 MHz request silently
 * keeps the clock selected by the first stream after boot. */
static void s31_gpio_diag_release(void) {
  s31_gpio_diag_stop();
  s_s31_parlio_delimiter = NULL;
  if (s_s31_parlio_delimiter_half != NULL) {
    ESP_ERROR_CHECK(
        parlio_del_rx_delimiter(s_s31_parlio_delimiter_half));
    s_s31_parlio_delimiter_half = NULL;
  }
  if (s_s31_parlio_delimiter_full != NULL) {
    ESP_ERROR_CHECK(
        parlio_del_rx_delimiter(s_s31_parlio_delimiter_full));
    s_s31_parlio_delimiter_full = NULL;
  }
  if (s_s31_parlio_unit != NULL) {
    ESP_ERROR_CHECK(parlio_del_rx_unit(s_s31_parlio_unit));
    s_s31_parlio_unit = NULL;
  }
  s_s31_parlio_data_width = 0u;
  s_s31_parlio_rate_hz = 0u;
  s_s31_parlio_swapped_pins = false;
#if !(CONFIG_ESP_SDR_TRANSPORT_USB && CONFIG_ESP_SDR_TRANSPORT_ETHERNET)
  heap_caps_free(s_s31_parlio_dma_buffer);
#endif
  s_s31_parlio_dma_buffer = NULL;
  for (uint32_t bit = 0u; bit < 16u; ++bit) {
    if (s_s31_diag_gpio[bit] >= 0) {
      ESP_ERROR_CHECK(gpio_reset_pin((gpio_num_t)s_s31_diag_gpio[bit]));
    }
  }
}

static void engine_enable(void) {
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_ENABLE_BIT);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                    MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);

#endif
  modem_enter_debug_mode();
#if CONFIG_IDF_TARGET_ESP32S31
  capture_config_t engine_config = s31_engine_config(&s_config);
  s31_gpio_diag_prepare(&s_config);
  s31_poison_dump_window();
  /* PHY/I2C configuration has already completed with the dump switch closed.
   * S31 only latches the register-only reset and dump setup while it is open. */
  esp_ipc_isr_stall_other_cpu();
  taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                   S31_ADCTRIG_TCM_DUMP_CTRL);
  __asm__ __volatile__("fence" ::: "memory");
#endif
  adctrig_prepare(DUMP_BANK_WORDS, &engine_config); /* sample_count = one bank */
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_ENABLE_BIT);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                    MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
  reg32p_set_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                  MODEM_WIFI_DUMP_CTRL_ENABLE_BIT);
#if CONFIG_IDF_TARGET_ESP32S31
  uint32_t setup_dwell_sel =
      (s_config.iq_engine.adc_source_sel & S31_ADC_SOURCE_PULSE_DWELL_M) >>
      S31_ADC_SOURCE_PULSE_DWELL_S;
  uint32_t setup_dwell = 64u << setup_dwell_sel;
  for (volatile uint32_t i = 0u; i < setup_dwell; ++i) {
    __asm__ __volatile__("nop");
  }
  if ((s_config.iq_engine.adc_source_sel & S31_ADC_SOURCE_TCM_ALWAYS_ON) ==
      0u) {
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  }
  __asm__ __volatile__("fence" ::: "memory");
  taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  esp_ipc_isr_release_other_cpu();
#endif
}

static void engine_disable(void) {
#if CONFIG_IDF_TARGET_ESP32S31
  /* Preserve PARLIO's internal stash/DMA allocations across ordinary capture
   * transitions. prepare() releases them only if immutable geometry changes;
   * repeated delete/new cycles fragment the small internal heap. */
  s31_gpio_diag_stop();
#endif
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_ENABLE_BIT |
                        MODEM_WIFI_DUMP_CTRL_CONTINUOUS_TRIGGER_GATE_BIT);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                    MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
#endif
  __asm__ __volatile__("fence" ::: "memory");
}

/* ---- triggers ---- */

static inline bool trigger_interval_select(const trigger_plan_t *plan,
                                           uint32_t source_chunk_index) {
  if (plan->interval_chunks == 0u || plan->interval_duration_chunks == 0u) {
    return true;
  }
  if (source_chunk_index < plan->interval_offset_chunks) {
    return false;
  }
  uint32_t position = (source_chunk_index - plan->interval_offset_chunks) %
                      plan->interval_chunks;
  return position < plan->interval_duration_chunks;
}

static uint32_t IRAM_ATTR trigger_interval_next_selected(
    const trigger_plan_t *plan, uint32_t source_chunk_index) {
  if (plan->interval_chunks == 0u || plan->interval_duration_chunks == 0u) {
    return source_chunk_index;
  }
  if (source_chunk_index < plan->interval_offset_chunks) {
    return plan->interval_offset_chunks;
  }

  uint32_t since_offset = source_chunk_index - plan->interval_offset_chunks;
  uint32_t position = since_offset % plan->interval_chunks;
  if (position < plan->interval_duration_chunks) {
    return source_chunk_index;
  }
  return source_chunk_index + (plan->interval_chunks - position);
}

static uint32_t IRAM_ATTR trigger_interval_selected_count(
    const trigger_plan_t *plan, uint32_t first_chunk, uint32_t end_chunk) {
  if (end_chunk <= first_chunk) {
    return 0u;
  }
  if (plan->interval_chunks == 0u || plan->interval_duration_chunks == 0u) {
    return end_chunk - first_chunk;
  }
  uint32_t duration = plan->interval_duration_chunks;
  if (duration > plan->interval_chunks) {
    duration = plan->interval_chunks;
  }

  uint64_t before_first = 0u;
  uint64_t before_end = 0u;
  if (first_chunk > plan->interval_offset_chunks) {
    uint32_t rel = first_chunk - plan->interval_offset_chunks;
    before_first = (uint64_t)(rel / plan->interval_chunks) * duration;
    uint32_t rem = rel % plan->interval_chunks;
    before_first += rem < duration ? rem : duration;
  }
  if (end_chunk > plan->interval_offset_chunks) {
    uint32_t rel = end_chunk - plan->interval_offset_chunks;
    before_end = (uint64_t)(rel / plan->interval_chunks) * duration;
    uint32_t rem = rel % plan->interval_chunks;
    before_end += rem < duration ? rem : duration;
  }
  uint64_t count = before_end - before_first;
  return count > UINT32_MAX ? UINT32_MAX : (uint32_t)count;
}

static bool IRAM_ATTR trigger_interval_prev_selected(const trigger_plan_t *plan,
                                                     uint32_t end_chunk,
                                                     uint32_t *selected_chunk) {
  if (end_chunk == 0u) {
    return false;
  }
  if (plan->interval_chunks == 0u || plan->interval_duration_chunks == 0u) {
    *selected_chunk = end_chunk - 1u;
    return true;
  }
  uint32_t duration = plan->interval_duration_chunks;
  if (duration > plan->interval_chunks) {
    duration = plan->interval_chunks;
  }
  if (end_chunk <= plan->interval_offset_chunks) {
    return false;
  }
  uint32_t rel_end = end_chunk - plan->interval_offset_chunks;
  uint32_t period_index = (rel_end - 1u) / plan->interval_chunks;
  uint32_t period_start =
      plan->interval_offset_chunks + period_index * plan->interval_chunks;
  uint32_t selected_end = period_start + duration;
  if (selected_end > end_chunk) {
    selected_end = end_chunk;
  }
  if (selected_end > period_start) {
    *selected_chunk = selected_end - 1u;
    return true;
  }
  if (period_index == 0u) {
    return false;
  }
  period_start -= plan->interval_chunks;
  *selected_chunk = period_start + duration - 1u;
  return true;
}

static trigger_plan_t build_trigger_plan(const capture_config_t *config) {
  trigger_plan_t plan = {
      .mode = config->trigger.trigger_mode,
      .interval_chunks = config->trigger.trigger_config[0], /* now in CHUNKS */
      .interval_duration_chunks = config->trigger.trigger_config[2],
      .checks_per_chunk = 1u,
  };
  if (plan.interval_chunks == 0u) {
    plan.interval_chunks = 1u;
  }
  plan.interval_offset_chunks =
      config->trigger.trigger_config[1] % plan.interval_chunks;

  if (config->trigger.trigger_mode == IQ_TRIGGER_MODE_AGC) {
    plan.checks_per_chunk = config->trigger.trigger_config[0];
    plan.pre_chunks = config->trigger.trigger_config[1];
    plan.post_chunks = config->trigger.trigger_config[2];
    plan.max_gain = config->trigger.trigger_config[3];
    plan.state_mask =
        config->trigger.trigger_config[4] & IQ_AGC_TRIGGER_STATE_MASK_ALL;
    uint32_t match_mode = config->trigger.trigger_config[5];
    plan.use_fsm_match = match_mode != IQ_AGC_TRIGGER_MATCH_MAX_GAIN;
    plan.use_max_gain_match = match_mode != IQ_AGC_TRIGGER_MATCH_FSM;
  } else if (config->trigger.trigger_mode == IQ_TRIGGER_MODE_POWER) {
    plan.checks_per_chunk = config->trigger.trigger_config[0];
    plan.pre_chunks = config->trigger.trigger_config[1];
    plan.post_chunks = config->trigger.trigger_config[2];
    plan.threshold = config->trigger.trigger_config[3];
    plan.dc_shift = config->trigger.trigger_config[4];
  }
  if (plan.checks_per_chunk == 0u) {
    plan.checks_per_chunk = 1u;
  }
  if (plan.checks_per_chunk > IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK) {
    plan.checks_per_chunk = IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK;
  }
  uint32_t sample_stride = IQ_CHUNK_SAMPLE_WORDS / plan.checks_per_chunk;
  if (sample_stride == 0u) {
    sample_stride = 1u;
  }
  for (uint32_t check = 0u; check < plan.checks_per_chunk; ++check) {
    plan.sample_offsets[check] = (uint16_t)(check * sample_stride);
  }
  return plan;
}

/* Scan one chunk's samples; return true if it triggers (AGC / power). */
static bool IRAM_ATTR chunk_triggers(const trigger_plan_t *plan,
                                     const uint32_t *words, int32_t *sum_i,
                                     int32_t *sum_q, uint32_t *sum_n,
                                     int32_t dc_i, int32_t dc_q,
                                     bool dc_valid) {
  bool found = false;
  for (uint32_t check = 0u; check < plan->checks_per_chunk; ++check) {
    uint32_t raw = words[plan->sample_offsets[check]];
    if (plan->mode == IQ_TRIGGER_MODE_AGC) {
      uint32_t agc_state = raw >> 28;
      uint32_t rx_gain = (raw >> 20) & 0xffu;
      bool fsm = (plan->state_mask & (1u << agc_state)) != 0u;
      bool maxg = rx_gain <= plan->max_gain;
      if ((!plan->use_fsm_match || fsm) &&
          (!plan->use_max_gain_match || maxg)) {
        found = true;
      }
    } else { /* power */
      int32_t i = sign_extend_10_u32(raw);
      int32_t q = sign_extend_10_u32(raw >> 10);
      *sum_i += i;
      *sum_q += q;
      ++*sum_n;
      uint32_t mag = abs_i32_u32(i - dc_i) + abs_i32_u32(q - dc_q);
      if (dc_valid && mag >= plan->threshold) {
        found = true;
      }
    }
  }
  return found;
}

static void IRAM_ATTR expand_pre_post(uint8_t *mask, uint32_t n, uint32_t pre,
                                      uint32_t post) {
  uint32_t last_trig = UINT32_MAX;
  for (uint32_t i = 0u; i < n; ++i) {
    if (mask[i] == CHUNK_STREAM_TRIGGER) {
      last_trig = i;
      continue;
    }
    if (last_trig != UINT32_MAX && i - last_trig <= post &&
        mask[i] == CHUNK_STREAM_SKIP) {
      mask[i] = CHUNK_STREAM_POST;
    }
  }
  uint32_t next_trig = UINT32_MAX;
  for (uint32_t j = n; j > 0u; --j) {
    uint32_t i = j - 1u;
    if (mask[i] == CHUNK_STREAM_TRIGGER) {
      next_trig = i;
      continue;
    }
    if (next_trig != UINT32_MAX && next_trig - i <= pre &&
        mask[i] == CHUNK_STREAM_SKIP) {
      mask[i] = CHUNK_STREAM_PRE;
    }
  }
}

/* ---- producer: turn a frozen bank's [lo,hi) into aligned chunks ---- */

/* DCOC tap and feed-phase strides are odd and therefore coprime to the
 * power-of-two output chunk.  Rotating 32 well-spread taps gives the EMA an
 * unbiased traversal of periodic waveforms without adding a full-chunk scan
 * to the already-unavoidable S31 writer-off handoff. */
#define DCOC_TAP_STRIDE_WORDS 113u
#define DCOC_PHASE_STEP_WORDS 239u
static uint32_t s_dcoc_sample_phase;

static void IRAM_ATTR dcoc_feed_chunk(const volatile uint32_t *words) {
  if (!dcoc_feed_due()) {
    return;
  }
  int32_t sum_i = 0, sum_q = 0;
  _Static_assert((IQ_CHUNK_SAMPLE_WORDS & (IQ_CHUNK_SAMPLE_WORDS - 1u)) == 0u,
                 "DCOC phase wrap requires a power-of-two chunk");
  for (uint32_t j = 0u; j < DCOC_SAMPLES_PER_FEED; ++j) {
    uint32_t sample =
        (s_dcoc_sample_phase + j * DCOC_TAP_STRIDE_WORDS) &
        (IQ_CHUNK_SAMPLE_WORDS - 1u);
    uint32_t word = words[sample];
    sum_i += sign_extend_10_u32(word);
    sum_q += sign_extend_10_u32(word >> 10);
  }
  s_dcoc_sample_phase =
      (s_dcoc_sample_phase + DCOC_PHASE_STEP_WORDS) &
      (IQ_CHUNK_SAMPLE_WORDS - 1u);
  dcoc_feed(sum_i, sum_q);
}

static void IRAM_ATTR fill_and_push_chunk(const capture_config_t *config,
                                          const volatile uint32_t *words,
                                          uint32_t source_chunk,
                                          uint32_t sample_rate_hz,
                                          uint32_t rx_gain, uint32_t agc_state,
                                          stream_frame_t *slot) {
  uint32_t late_misses = 0u;
  uint32_t write_ptr = s_dbg_lo;
  uint32_t producer_write_ptr = adc_dump_store_addr();
#if CONFIG_IDF_TARGET_ESP32S31
  if (s31_pulse_tcm_before_copy(config)) {
    late_misses = s_s31_copy_ctrl_diag;
    write_ptr = s_s31_copy_mode_before_diag;
    producer_write_ptr =
        S31_PULSE_META_MARKER | ((s_s31_copy_physical_chunk & 0xffu) << 16) |
        (dump_ring_mod(
             adc_dump_store_addr_from_raw(s_s31_copy_mode_after_diag)) &
         0xffffu);
  }
#endif
  iq_chunk_report_meta_t meta = {
      .source_chunk_index = source_chunk,
      .adc_decimation = adc_decimation_value(config),
      .sample_rate_hz = sample_rate_hz,
      .center_freq_mhz = modem_rf_freq_hz() / HZ_PER_MHZ,
      .rx_gain = rx_gain,
      .agc_state = agc_state,
      .dropped_chunks = s_stream_dropped_chunks,
      .bank_timer_late_misses = late_misses,
      .bank_timer_write_ptr = write_ptr,
      .producer_wake_write_ptr = producer_write_ptr,
  };
  if (stream_output_int8()) {
    if ((config->rx_filter.rx_filter_override >= 2u &&
         config->rx_filter.rx_filter_override < 8u) ||
        config->rx_filter.rx_filter_override == 28u ||
        config->rx_filter.rx_filter_override == 29u ||
        config->rx_filter.rx_filter_override == 30u ||
        config->rx_filter.rx_filter_override == 31u ||
        config->rx_filter.rx_filter_override == 32u ||
        config->rx_filter.rx_filter_override == 33u ||
        config->rx_filter.rx_filter_override == 34u ||
        config->rx_filter.rx_filter_override == 35u ||
        config->rx_filter.rx_filter_override == 36u ||
        config->rx_filter.rx_filter_override == 37u ||
        config->rx_filter.rx_filter_override == 14u ||
        config->rx_filter.rx_filter_override == 15u) {
      stream_ring_fill_bt_bytes_chunk_int8(slot, &meta, words);
    } else {
      stream_ring_fill_iq_chunk_int8(slot, &meta, words);
    }
  } else {
    stream_ring_fill_iq_chunk(slot, &meta, words);
  }
  dcoc_feed_chunk(words);
}

#if CONFIG_IDF_TARGET_ESP32S31
static void IRAM_ATTR fill_and_push_chunk_strided(
    const capture_config_t *config, uint32_t source_chunk,
    uint32_t sample_rate_hz, uint32_t rx_gain, uint32_t agc_state,
    stream_frame_t *slot, uint32_t start_word, uint32_t stride_words) {
  iq_chunk_report_meta_t meta = {
      .source_chunk_index = source_chunk,
      .adc_decimation = adc_decimation_value(config),
      .sample_rate_hz = sample_rate_hz,
      .center_freq_mhz = modem_rf_freq_hz() / HZ_PER_MHZ,
      .rx_gain = rx_gain,
      .agc_state = agc_state,
      .dropped_chunks = s_stream_dropped_chunks,
      .bank_timer_late_misses = s_s31_copy_ctrl_diag,
      .bank_timer_write_ptr = s_s31_copy_mode_before_diag,
      .producer_wake_write_ptr =
          S31_PULSE_META_MARKER |
          (((start_word / IQ_CHUNK_SAMPLE_WORDS) & 0xffu) << 16) |
          (dump_ring_mod(adc_dump_store_addr_from_raw(
               s_s31_copy_mode_after_diag)) &
           0xffffu),
  };
  if (stream_output_int8()) {
    if ((config->rx_filter.rx_filter_override >= 2u &&
         config->rx_filter.rx_filter_override < 8u) ||
        config->rx_filter.rx_filter_override == 28u ||
        config->rx_filter.rx_filter_override == 29u ||
        config->rx_filter.rx_filter_override == 30u ||
        config->rx_filter.rx_filter_override == 31u ||
        config->rx_filter.rx_filter_override == 32u ||
        config->rx_filter.rx_filter_override == 33u ||
        config->rx_filter.rx_filter_override == 34u ||
        config->rx_filter.rx_filter_override == 35u ||
        config->rx_filter.rx_filter_override == 36u ||
        config->rx_filter.rx_filter_override == 37u ||
        config->rx_filter.rx_filter_override == 14u ||
        config->rx_filter.rx_filter_override == 15u) {
      stream_ring_fill_bt_bytes_chunk_strided_int8(
          slot, &meta, DUMP_BASE, start_word, stride_words, DUMP_BANK_WORDS);
    } else {
      stream_ring_fill_iq_chunk_strided_int8(
          slot, &meta, DUMP_BASE, start_word, stride_words, DUMP_BANK_WORDS);
    }
  } else {
    stream_ring_fill_iq_chunk_strided(slot, &meta, DUMP_BASE, start_word,
                                      stride_words, DUMP_BANK_WORDS);
  }
  /* DCOC servo feed: retrace the strided ring walk the fill above used
   * (slot->iq is packed, so the copied samples can't be pointed at). */
  if (dcoc_feed_due()) {
    uint32_t source = dump_ring_mod(
        start_word + s_dcoc_sample_phase * stride_words);
    uint32_t step = DCOC_TAP_STRIDE_WORDS * stride_words;
    int32_t sum_i = 0, sum_q = 0;
    for (uint32_t j = 0u; j < DCOC_SAMPLES_PER_FEED; ++j) {
      uint32_t word = DUMP_BASE[source];
      sum_i += sign_extend_10_u32(word);
      sum_q += sign_extend_10_u32(word >> 10);
      source += step;
      while (source >= DUMP_BANK_WORDS) {
        source -= DUMP_BANK_WORDS;
      }
    }
    s_dcoc_sample_phase =
        (s_dcoc_sample_phase + DCOC_PHASE_STEP_WORDS) &
        (IQ_CHUNK_SAMPLE_WORDS - 1u);
    dcoc_feed(sum_i, sum_q);
  }
}
#endif

static void stream_state_reset(void) {
  stream_ring_reset();
  s_stream_dropped_chunks = 0u;
  s_source_chunk_index = 0u;
  s_emit_chunk = 0u;
  s_s31_ring_started = false;
#if CONFIG_IDF_TARGET_ESP32S31
  s_s31_diag_abs_write = 0u;
  s_s31_diag_c_hi = 0u;
  s_s31_diag_emit_chunk = 0u;
  s_s31_diag_ring_started = 0u;
  s_s31_abs_write_accum = 0u;
  s_s31_abs_track_us = 0;
  s_s31_stage_valid = false;
  s_s31_stage_source_chunk = 0u;
  s_s31_stage_prev_end_raw = 0u;
  s_s31_stage_have_prev_end = false;
  s_s31_micro_gate_advances = 0u;
  s_s31_micro_gate_max_cycles = 0u;
  s_s31_micro_gate_batches = 0u;
  s_s31_cursor_min_backlog = UINT32_MAX;
  s_s31_cursor_max_backlog = 0u;
  s_s31_cursor_batches = 0u;
  s_s31_cursor_last_writer_cycle = 0u;
  s_s31_cursor_last_backlog = 0u;
  s_s31_cursor_have_writer_cycle = false;
  s_s31_cursor_overruns = 0u;
  s_s31_probe_source_chunk = 0u;
  s_s31_probe_end_cycle = 0u;
  s_s31_probe_have_end_cycle = false;
  s_s31_probe_logged_selector = UINT32_MAX;
#endif
  s_power_trigger_dc_valid = false;
  s_power_trigger_dc_i_q16 = 0;
  s_power_trigger_dc_q_q16 = 0;
  s_isr_prev_sa = adc_dump_store_addr();
  s_isr_wrap_count = 0u;
}

static void wait_stream_pipeline_idle(void) {
  for (uint32_t i = 0u;
       i < 100u && (s_stream_write_active || s_producer_active); ++i) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

#if CONFIG_IDF_TARGET_ESP32S31
#define S31_RING_GUARD_WORDS (12u * IQ_CHUNK_SAMPLE_WORDS)
#define S31_LIVE_COPY_GUARD_WORDS 256u
#define S31_RING_MAX_CHUNKS_PER_POLL 16u
#define S31_POLL_CLOSE_US 20u
#define S31_POLL_WAKE_MARGIN_US 30000u
#define S31_STAGE_WORDS (14u * IQ_CHUNK_SAMPLE_WORDS)
#define S31_STAGE_CHUNKS (S31_STAGE_WORDS / IQ_CHUNK_SAMPLE_WORDS)
#define S31_GPIO_DIAG_CHUNKS 2u
/* The live writer wraps at the programmed, chunk-aligned aperture. */
static volatile uint32_t *const S31_STAGE_BASE =
    (volatile uint32_t *)S31_MAC_DUMP_SRAM_GUARD_START;
static gdma_channel_handle_t s_s31_tcm_dma_tx[2];
static gdma_channel_handle_t s_s31_tcm_dma_rx[2];
static gdma_link_list_handle_t s_s31_tcm_dma_tx_links[2];
static gdma_link_list_handle_t s_s31_tcm_dma_rx_links[2];
static size_t s_s31_tcm_dma_int_align[2];
static size_t s_s31_tcm_dma_ext_align[2];
static int s_s31_tcm_dma_channel[2] = {-1, -1};
static gptimer_handle_t s_s31_tcm_gate_watchdog;
static RTC_NOINIT_ATTR volatile bool s_s31_tcm_gate_watchdog_fired;
/* The modem owns the complete TCM fabric while its dump gate is open; even
 * stores to the nominal guard can stall.  Live diagnostic-bus sampling must
 * therefore land in ordinary HP SRAM. */
static RTC_NOINIT_ATTR __attribute__((aligned(64))) uint32_t
    s_s31_gpio_diag_stage[S31_GPIO_DIAG_CHUNKS * IQ_CHUNK_SAMPLE_WORDS];
extern void s31_pie_memcpy_aligned(void *dst, const void *src,
                                   size_t byte_count);
extern uint32_t s31_pie_microgate_copy8(void *dst, const void *src,
                                       volatile uint32_t *gate_reg,
                                       uint32_t gate_mask);
extern uint32_t s31_pie_microgate_copy32(void *dst, const void *src,
                                        volatile uint32_t *gate_reg,
                                        uint32_t gate_mask);
_Static_assert(S31_STAGE_WORDS * sizeof(uint32_t) <=
                   MAC_DUMP_SRAM_BANK_BYTES,
               "pipelined staging area must fit the 64 KiB S31 guard");

static esp_err_t s31_tcm_dma_prepare(uint32_t backend, void *destination,
                                     size_t byte_count) {
  if (s_s31_tcm_dma_tx[backend] == NULL) {
    gdma_channel_alloc_config_t alloc_config = {0};
    esp_err_t err = backend == 0u
                        ? gdma_new_axi_channel(&alloc_config,
                                               &s_s31_tcm_dma_tx[backend],
                                               &s_s31_tcm_dma_rx[backend])
                        : gdma_new_ahb_channel(&alloc_config,
                                               &s_s31_tcm_dma_tx[backend],
                                               &s_s31_tcm_dma_rx[backend]);
    if (err != ESP_OK) {
      return err;
    }
    gdma_strategy_config_t strategy = {
        .owner_check = true,
        .auto_update_desc = true,
        .eof_till_data_popped = true,
    };
    ESP_RETURN_ON_ERROR(gdma_apply_strategy(s_s31_tcm_dma_tx[backend],
                                             &strategy),
                        "iq_capture", "TCM DMA TX strategy");
    ESP_RETURN_ON_ERROR(gdma_apply_strategy(s_s31_tcm_dma_rx[backend],
                                             &strategy),
                        "iq_capture", "TCM DMA RX strategy");
    gdma_transfer_config_t transfer = {
        .max_data_burst_size = 16u,
        .access_ext_mem = true,
    };
    ESP_RETURN_ON_ERROR(gdma_config_transfer(s_s31_tcm_dma_tx[backend],
                                              &transfer),
                        "iq_capture", "TCM DMA TX transfer");
    ESP_RETURN_ON_ERROR(gdma_config_transfer(s_s31_tcm_dma_rx[backend],
                                              &transfer),
                        "iq_capture", "TCM DMA RX transfer");
    gdma_trigger_t trigger = GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_M2M, 0);
    uint32_t free_mask = 0u;
    ESP_RETURN_ON_ERROR(gdma_get_free_m2m_trig_id_mask(
                            s_s31_tcm_dma_tx[backend], &free_mask),
                        "iq_capture", "TCM DMA trigger mask");
    if (free_mask == 0u) {
      return ESP_ERR_NOT_FOUND;
    }
    trigger.instance_id = __builtin_ctz(free_mask);
    ESP_RETURN_ON_ERROR(gdma_connect(s_s31_tcm_dma_tx[backend], trigger),
                        "iq_capture", "TCM DMA TX connect");
    ESP_RETURN_ON_ERROR(gdma_connect(s_s31_tcm_dma_rx[backend], trigger),
                        "iq_capture", "TCM DMA RX connect");
    gdma_channel_alignment_info_t alignment = {0};
    ESP_RETURN_ON_ERROR(gdma_get_channel_alignment_constraints(
                            s_s31_tcm_dma_tx[backend], &alignment),
                        "iq_capture", "TCM DMA alignment");
    s_s31_tcm_dma_int_align[backend] = alignment.int_mem_alignment;
    s_s31_tcm_dma_ext_align[backend] = alignment.ext_enc_mem_alignment;
    ESP_RETURN_ON_ERROR(gdma_get_channel_id(s_s31_tcm_dma_rx[backend],
                                             &s_s31_tcm_dma_channel[backend]),
                        "iq_capture", "TCM DMA channel ID");
    gdma_link_list_config_t link_config = {
        .num_items = 4u,
        .item_alignment = backend == 0u ? 8u : 4u,
        .flags = {
            .items_in_ext_mem = false,
            .check_owner = true,
        },
    };
    ESP_RETURN_ON_ERROR(gdma_new_link_list(
                            &link_config, &s_s31_tcm_dma_tx_links[backend]),
                        "iq_capture", "TCM DMA TX links");
    ESP_RETURN_ON_ERROR(gdma_new_link_list(
                            &link_config, &s_s31_tcm_dma_rx_links[backend]),
                        "iq_capture", "TCM DMA RX links");
  }

  ESP_RETURN_ON_ERROR(gdma_reset(s_s31_tcm_dma_tx[backend]), "iq_capture",
                      "TCM DMA TX reset");
  ESP_RETURN_ON_ERROR(gdma_reset(s_s31_tcm_dma_rx[backend]), "iq_capture",
                      "TCM DMA RX reset");
  gdma_buffer_mount_config_t tx_buffer = {
      .buffer = (void *)DUMP_BASE,
      .buffer_alignment = s_s31_tcm_dma_int_align[backend],
      .length = byte_count,
      .flags = {
          .mark_eof = true,
          .mark_final = GDMA_FINAL_LINK_TO_NULL,
          .bypass_buffer_addr_align_check = true,
      },
  };
  gdma_buffer_mount_config_t rx_buffer = {
      .buffer = destination,
      .buffer_alignment = s_s31_tcm_dma_int_align[backend],
      .length = byte_count,
      .flags = {
          .mark_final = GDMA_FINAL_LINK_TO_NULL,
          .bypass_buffer_addr_align_check = true,
      },
  };
  ESP_RETURN_ON_ERROR(gdma_link_mount_buffers(
                          s_s31_tcm_dma_tx_links[backend], 0, &tx_buffer, 1,
                          NULL),
                      "iq_capture", "TCM DMA mount TX");
  ESP_RETURN_ON_ERROR(gdma_link_mount_buffers(
                          s_s31_tcm_dma_rx_links[backend], 0, &rx_buffer, 1,
                          NULL),
                      "iq_capture", "TCM DMA mount RX");
  return ESP_OK;
}

static bool IRAM_ATTR s31_tcm_gate_watchdog_cb(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *event,
    void *user_data) {
  (void)timer;
  (void)event;
  (void)user_data;
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  __asm__ __volatile__("fence rw, rw" ::: "memory");
  s_s31_tcm_gate_watchdog_fired = true;
  return false;
}

/* Called by app_main on core 0 so the watchdog interrupt can recover core 1
 * even when a DMA start transaction stalls that core's bus interface. */
static void s31_tcm_gate_watchdog_init(void) {
  if (s_s31_tcm_gate_watchdog != NULL) {
    return;
  }
  gptimer_config_t config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000u,
      .intr_priority = 3,
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&config, &s_s31_tcm_gate_watchdog));
  gptimer_event_callbacks_t callbacks = {
      .on_alarm = s31_tcm_gate_watchdog_cb,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(
      s_s31_tcm_gate_watchdog, &callbacks, NULL));
  ESP_ERROR_CHECK(gptimer_enable(s_s31_tcm_gate_watchdog));
}

typedef struct {
  uint32_t source_chunk;
  uint32_t start_raw;
  uint32_t end_raw;
} s31_stage_batch_diag_t;

static void s31_prepare_stage_frames(const capture_config_t *config,
                                     stream_frame_t **slots, uint32_t n,
                                     const s31_stage_batch_diag_t *diag) {
  uint32_t sample_cycles = adc_dump_sample_cycles(config);
  uint32_t sample_rate_hz =
      sample_cycles != 0u ? ADC_DUMP_CLOCK_HZ / sample_cycles : 0u;
  uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
  uint32_t rx_gain = agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M;
  uint32_t agc_state = (agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                       MODEM_WIFI_AGC_AGCRD3_STATE_S;
  uint32_t ctrl = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL);
  for (uint32_t frame = 0u; frame < n; ++frame) {
    uint32_t batch = frame / S31_STAGE_CHUNKS;
    uint32_t source_chunk =
        diag[batch].source_chunk + (frame % S31_STAGE_CHUNKS);
    iq_chunk_report_meta_t meta = {
        .source_chunk_index = source_chunk,
        .adc_decimation = adc_decimation_value(config),
        .sample_rate_hz = sample_rate_hz,
        .center_freq_mhz = modem_rf_freq_hz() / HZ_PER_MHZ,
        .rx_gain = rx_gain,
        .agc_state = agc_state,
        .dropped_chunks = s_stream_dropped_chunks,
        .bank_timer_late_misses = ctrl,
        .bank_timer_write_ptr = diag[batch].start_raw,
        .producer_wake_write_ptr =
            S31_PULSE_META_MARKER |
            dump_ring_mod(diag[batch].end_raw),
    };
    stream_ring_prepare_iq_chunk(slots[frame], &meta);
  }
}

#if CONFIG_ESP_SDR_TRANSPORT_USB
/* All static IQ fields and IQU transport tickets are prepared before the TCM
 * switch opens. Core 1 has write-through PSRAM attributes from startup, so
 * these measured boundary fields and CRCs become DMA-visible without a cache
 * operation. This routine deliberately contains no RTOS, cache, or USB-stack
 * calls and is safe while core 0 is parked. */
static void __attribute__((noinline)) s31_finalize_usb_stage_batch(
    stream_frame_t **slots, uint32_t n,
    const s31_stage_batch_diag_t *diag) {
  for (uint32_t frame = 0u; frame < n; ++frame) {
    uint32_t batch = frame / S31_STAGE_CHUNKS;
    uint32_t source_chunk =
        diag[batch].source_chunk + (frame % S31_STAGE_CHUNKS);
    iq_chunk_t *iq = &slots[frame]->iq;
    iq->source_chunk_index = source_chunk;
    iq->chunk_counter = source_chunk;
    iq->dropped_chunks = s_stream_dropped_chunks;
    iq->bank_timer_write_ptr = diag[batch].start_raw;
    iq->producer_wake_write_ptr =
        S31_PULSE_META_MARKER |
        dump_ring_mod(diag[batch].end_raw);

    uint8_t *packet = (uint8_t *)slots[frame] - sizeof(iq_udp_header_t);
    iq_udp_header_t *header = (iq_udp_header_t *)packet;
    header->frame_sequence = iq->sequence;
    header->source_chunk_index = source_chunk;
    header->frame_crc32 = iq->crc32;
    header->firmware_dropped_chunks = s_stream_dropped_chunks;
    memcpy(header->frame_magic, iq->magic, sizeof(header->frame_magic));
    header->header_crc32 = esp_rom_crc32_le(
        0u, packet, offsetof(iq_udp_header_t, header_crc32));
  }
  __asm__ __volatile__("fence rw, rw" ::: "memory");
}

static bool s31_modem_diag_probe_enabled(const capture_config_t *config) {
  uint32_t expert = config->rx_filter.rx_filter_override;
  return (expert >= S31_MODEM_DIAG_PROBE_OVERRIDE_FIRST &&
          expert <= S31_MODEM_DIAG_PROBE_OVERRIDE_LAST) ||
         (expert >= S31_DATADUMP_MMIO_PROBE_OVERRIDE_FIRST &&
          expert <= S31_DATADUMP_MMIO_PROBE_OVERRIDE_LAST) ||
         expert == S31_LP_CORE_PROBE_OVERRIDE ||
         expert == S31_HP_TCM_PROBE_OVERRIDE ||
         expert == S31_SEAMLESS_MICRO_GATE_OVERRIDE ||
         expert == S31_GPIO_DIAG_OVERRIDE ||
         expert == S31_PARLIO_NATIVE_IQ_OVERRIDE ||
         expert == S31_PARLIO_HOST_IQ_OVERRIDE;
}

/* Diagnostic proof-of-path for the live modem bus.  Sampling into the lower
 * reserved TCM guard avoids PSRAM latency and, unlike the ADC dump aperture,
 * requires no TCM ownership switch.  The later TCM-to-PSRAM copy deliberately
 * leaves an honestly reported inter-burst gap; once a valid IQ selector is
 * found, the next step is to attach this bus to a continuous DMA sink. */
static void s31_process_modem_diag_probe(const capture_config_t *config) {
  uint32_t expert = config->rx_filter.rx_filter_override;
  const bool lp_core_probe = expert == S31_LP_CORE_PROBE_OVERRIDE;
  const bool hp_tcm_probe = expert == S31_HP_TCM_PROBE_OVERRIDE;
  const bool seamless_micro_gate =
      expert == S31_SEAMLESS_MICRO_GATE_OVERRIDE;
  const bool gpio_diag = expert == S31_GPIO_DIAG_OVERRIDE;
  const uint32_t capture_chunks =
      (gpio_diag || seamless_micro_gate) ? S31_GPIO_DIAG_CHUNKS
                                         : S31_STAGE_CHUNKS;
  const uint32_t capture_words = capture_chunks * IQ_CHUNK_SAMPLE_WORDS;
  stream_frame_t *slots[S31_STAGE_CHUNKS];
  if (!iq_usb_direct_reserve(capture_chunks, slots)) {
    return;
  }

  uint32_t sample_cycles = adc_dump_sample_cycles(config);
  uint32_t sample_rate_hz = sample_cycles != 0u
                                ? ADC_DUMP_CLOCK_HZ / sample_cycles
                                : 0u;
  if (sample_rate_hz == 0u) {
    iq_usb_direct_abort();
    return;
  }
  uint32_t cycles_per_sample =
      (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000u) / sample_rate_hz;
  if (cycles_per_sample == 0u) {
    cycles_per_sample = 1u;
  }

  uint32_t start_cycle = esp_cpu_get_cycle_count();
  if (s_s31_probe_have_end_cycle) {
    uint32_t idle_cycles = start_cycle - s_s31_probe_end_cycle;
    uint32_t missed_samples = idle_cycles > cycles_per_sample
                                  ? idle_cycles / cycles_per_sample - 1u
                                  : 0u;
    uint32_t missed_chunks =
        (missed_samples + IQ_CHUNK_SAMPLE_WORDS - 1u) /
        IQ_CHUNK_SAMPLE_WORDS;
    s_stream_dropped_chunks += missed_chunks;
    s_s31_probe_source_chunk += missed_chunks;
  }

  uint32_t selector = config->rx_filter.rx_filter_mode & 31u;
  s31_stage_batch_diag_t diag = {
      .source_chunk = s_s31_probe_source_chunk,
      .start_raw = selector,
      .end_raw = 0u,
  };
  s31_prepare_stage_frames(config, slots, capture_chunks, &diag);

  const bool datadump_mmio =
      expert >= S31_DATADUMP_MMIO_PROBE_OVERRIDE_FIRST &&
      expert <= S31_DATADUMP_MMIO_PROBE_OVERRIDE_LAST;
  const bool swap = !datadump_mmio && !lp_core_probe && !hp_tcm_probe &&
                    !gpio_diag &&
                    !seamless_micro_gate &&
                    ((expert - S31_MODEM_DIAG_PROBE_OVERRIDE_FIRST) & 1u) !=
                        0u;
  uint32_t probe_offset = datadump_mmio
                              ? 0x20u +
                                    (expert -
                                     S31_DATADUMP_MMIO_PROBE_OVERRIDE_FIRST) *
                                        0x80u +
                                    selector * sizeof(uint32_t)
                              : 0u;
  uintptr_t probe_addr = DR_REG_MODEM_DATADUMP_BASE + probe_offset;
  uint32_t deadline = esp_cpu_get_cycle_count();
  uint32_t first_sample_cycle = 0u;
  uint32_t last_sample_cycle = 0u;
  uint32_t previous = UINT32_MAX;
  uint32_t changes = 0u;
  uint32_t raw_min = UINT32_MAX;
  uint32_t raw_max = 0u;
  uint32_t late_samples = 0u;
  uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
  uint32_t status = ((agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M) << 20u) |
                    (((agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                      MODEM_WIFI_AGC_AGCRD3_STATE_S)
                     << 28u);
  uint32_t diag_fix_read =
      reg32_read_addr(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG);
  uint32_t diag_enable_read = reg32_read_addr(HP_SYSTEM_MODEM_DIAG_EN_REG);

  if (seamless_micro_gate) {
    /* Decisive bus-master experiment. Keep the destination in RTC SRAM and
     * use no DMA interrupt, so a blocked TCM read cannot deadlock PSRAM or an
     * ISR. Wait until this source range is known to have been written, start
     * the preconfigured DMA while modem ownership remains uninterrupted, and
     * sample the raw completion bit before returning ownership to the CPUs. */
    const uint32_t backend = 1u; /* AHB DMA keeps the PSRAM fabric independent. */
    memset((void *)DUMP_BASE, 0xa5, DUMP_BANK_WORDS * sizeof(uint32_t));
    memset(s_s31_gpio_diag_stage, 0x5a,
           capture_words * sizeof(uint32_t));
    esp_err_t prepare_err = s31_tcm_dma_prepare(
        backend, s_s31_gpio_diag_stage,
        capture_words * sizeof(uint32_t));
    if (prepare_err != ESP_OK) {
      ESP_LOGE("iq_capture", "TCM DMA backend %" PRIu32
                             " prepare failed: %s",
               backend, esp_err_to_name(prepare_err));
      iq_usb_direct_abort();
      return;
    }
    const int dma_channel = s_s31_tcm_dma_channel[backend];
    AHB_DMA.in_intr[dma_channel].clr.val = UINT32_MAX;
    AHB_DMA.out_intr[dma_channel].clr.val = UINT32_MAX;
    s_s31_tcm_gate_watchdog_fired = false;
    gptimer_alarm_config_t watchdog_alarm = {
        .alarm_count = 10000u,
    };
    ESP_ERROR_CHECK(gptimer_set_raw_count(s_s31_tcm_gate_watchdog, 0u));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_s31_tcm_gate_watchdog,
                                              &watchdog_alarm));
    ESP_ERROR_CHECK(gptimer_start(s_s31_tcm_gate_watchdog));
    uint32_t gate_mask = S31_ADCTRIG_TCM_DUMP_CTRL;
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, gate_mask);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    uint32_t last_ptr = dump_ring_mod(adc_dump_store_addr_raw());
    uint32_t writer_abs = 0u;
    first_sample_cycle = esp_cpu_get_cycle_count();
    while (writer_abs < capture_words + 512u &&
           !s_s31_tcm_gate_watchdog_fired) {
      uint32_t ptr = dump_ring_mod(adc_dump_store_addr_raw());
      uint32_t advance = dump_ring_delta(ptr, last_ptr);
      if (advance != 0u) {
        writer_abs += advance;
        last_ptr = ptr;
      }
    }
    esp_err_t rx_start = gdma_start(
        s_s31_tcm_dma_rx[backend],
        gdma_link_get_head_addr(s_s31_tcm_dma_rx_links[backend]));
    esp_err_t tx_start = gdma_start(
        s_s31_tcm_dma_tx[backend],
        gdma_link_get_head_addr(s_s31_tcm_dma_tx_links[backend]));
    bool dma_done_open = false;
    uint32_t dma_done_writer_abs = 0u;
    while (writer_abs < DUMP_BANK_WORDS + 1024u &&
           !s_s31_tcm_gate_watchdog_fired) {
      uint32_t ptr = dump_ring_mod(adc_dump_store_addr_raw());
      uint32_t advance = dump_ring_delta(ptr, last_ptr);
      if (advance != 0u) {
        writer_abs += advance;
        last_ptr = ptr;
      }
      uint32_t raw = AHB_DMA.in_intr[dma_channel].raw.val;
      if (!s_s31_tcm_gate_watchdog_fired && !dma_done_open &&
          (raw & BIT(1)) != 0u) {
        dma_done_open = true;
        dma_done_writer_abs = writer_abs;
      }
    }
    last_sample_cycle = esp_cpu_get_cycle_count();
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
    (void)gptimer_stop(s_s31_tcm_gate_watchdog);
    uint32_t post_raw = AHB_DMA.in_intr[dma_channel].raw.val;
    (void)gdma_stop(s_s31_tcm_dma_tx[backend]);
    (void)gdma_stop(s_s31_tcm_dma_rx[backend]);
    for (uint32_t word = 0u; word < capture_words; ++word) {
      uint32_t raw = s_s31_gpio_diag_stage[word];
      if (raw != 0xa5a5a5a5u && raw != 0x5a5a5a5au) {
        ++changes;
      }
      if (raw < raw_min) {
        raw_min = raw;
      }
      if (raw > raw_max) {
        raw_max = raw;
      }
    }
    late_samples = dma_done_open ? 0u : 1u;
    diag.start_raw = (backend << 31u) |
                     ((uint32_t)(rx_start == ESP_OK) << 30u) |
                     ((uint32_t)(tx_start == ESP_OK) << 29u) |
                     ((uint32_t)s_s31_tcm_gate_watchdog_fired << 28u) |
                     (dma_done_writer_abs & 0x0fffffffu);
    diag.end_raw = ((post_raw & 0xffu) << 24u) |
                   (changes & 0x00ffffffu);
  } else if (lp_core_probe) {
    /* Keep the modem writer enabled for the complete burst. Census the raw LP
     * probe at the target sample cadence; do not assume the load-result group
     * carries a software-generated valid bit until the hardware sweep proves
     * it. */
    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                     S31_ADCTRIG_TCM_DUMP_CTRL);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    deadline = esp_cpu_get_cycle_count();
    uint32_t sample = 0u;
    for (uint32_t frame = 0u; frame < capture_chunks; ++frame) {
      volatile uint32_t *dst = (volatile uint32_t *)(
          (uintptr_t)slots[frame] + offsetof(iq_chunk_t, samples));
      for (uint32_t word = 0u; word < IQ_CHUNK_SAMPLE_WORDS;
           ++word, ++sample) {
        uint32_t now;
        do {
          now = esp_cpu_get_cycle_count();
        } while ((int32_t)(now - deadline) < 0);
        uint32_t raw = reg32_read_addr(LP_SYSTEM_REG_LP_PROBE_OUT_REG);
        if (sample == 0u) {
          first_sample_cycle = now;
        } else if (now - deadline >= cycles_per_sample * 2u) {
          ++late_samples;
        }
        dst[word] = status | (raw & 0x000fffffu);
        if (raw != previous) {
          ++changes;
        }
        previous = raw;
        if (raw < raw_min) {
          raw_min = raw;
        }
        if (raw > raw_max) {
          raw_max = raw;
        }
        last_sample_cycle = now;
        deadline += cycles_per_sample;
      }
    }
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
    esp_ipc_isr_release_other_cpu();
  } else if (hp_tcm_probe) {
    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                     S31_ADCTRIG_TCM_DUMP_CTRL);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    deadline = esp_cpu_get_cycle_count();
    uint32_t sample = 0u;
    for (uint32_t frame = 0u; frame < capture_chunks; ++frame) {
      volatile uint32_t *dst = (volatile uint32_t *)(
          (uintptr_t)slots[frame] + offsetof(iq_chunk_t, samples));
      for (uint32_t word = 0u; word < IQ_CHUNK_SAMPLE_WORDS;
           ++word, ++sample) {
        uint32_t now;
        do {
          now = esp_cpu_get_cycle_count();
        } while ((int32_t)(now - deadline) < 0);
        if (sample == 0u) {
          first_sample_cycle = now;
        } else if (now - deadline >= cycles_per_sample) {
          ++late_samples;
        }
        uint32_t raw = reg32_read_addr(HP_SYSTEM_PROBE_OUT_REG);
        dst[word] = raw;
        if (raw != previous) {
          ++changes;
        }
        previous = raw;
        if (raw < raw_min) {
          raw_min = raw;
        }
        if (raw > raw_max) {
          raw_max = raw;
        }
        last_sample_cycle = now;
        deadline += cycles_per_sample;
      }
    }
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
    esp_ipc_isr_release_other_cpu();
  } else if (gpio_diag) {
    /* MODEM_DIAG_EN substitutes the modem's selected low 16-bit slice onto
     * HP_PROBE_TOP_OUT[15:0]. Loop those signals through GPIO pads which do
     * not overlap the module memory, console, or RGMII groups. */
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    deadline = esp_cpu_get_cycle_count();
    for (uint32_t sample = 0u; sample < capture_words; ++sample) {
      uint32_t now;
      do {
        now = esp_cpu_get_cycle_count();
      } while ((int32_t)(now - deadline) < 0);
      if (sample == 0u) {
        first_sample_cycle = now;
      } else if (now - deadline >= cycles_per_sample) {
        ++late_samples;
      }
      uint32_t raw = (GPIO.in.val >> 8u) & 0xffffu;
      int32_t i = (int8_t)(raw >> 8u);
      int32_t q = (int8_t)raw;
      s_s31_gpio_diag_stage[sample] =
          status | ((((uint32_t)(i << 2)) & 0x3ffu) << 10u) |
          (((uint32_t)(q << 2)) & 0x3ffu);
      if (raw != previous) {
        ++changes;
      }
      previous = raw;
      if (raw < raw_min) {
        raw_min = raw;
      }
      if (raw > raw_max) {
        raw_max = raw;
      }
      last_sample_cycle = now;
      deadline += cycles_per_sample;
    }
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  } else {
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    for (uint32_t sample = 0u; sample < capture_words; ++sample) {
    uint32_t now;
    do {
      now = esp_cpu_get_cycle_count();
    } while ((int32_t)(now - deadline) < 0);
    if (sample == 0u) {
      first_sample_cycle = now;
    } else if (now - deadline >= cycles_per_sample) {
      ++late_samples;
    }
    uint32_t raw = datadump_mmio
                       ? reg32_read_addr(probe_addr)
                       : reg32_read_addr(HP_SYSTEM_PROBE_OUT_REG) & 0xffffu;
    if (datadump_mmio) {
      /* Preserve the peripheral word verbatim.  A real sample port should
       * already use the dump engine's native Q[9:0]/I[19:10] packing. */
      S31_STAGE_BASE[sample] = raw;
    } else {
      uint32_t low = raw & 0xffu;
      uint32_t high = raw >> 8u;
      int32_t i = (int8_t)(swap ? low : high);
      int32_t q = (int8_t)(swap ? high : low);
      S31_STAGE_BASE[sample] = status |
                               (((uint32_t)i & 0x3ffu) << 10u) |
                               ((uint32_t)q & 0x3ffu);
    }
    if (raw != previous) {
      ++changes;
    }
    previous = raw;
    if (raw < raw_min) {
      raw_min = raw;
    }
    if (raw > raw_max) {
      raw_max = raw;
    }
    last_sample_cycle = now;
    deadline += cycles_per_sample;
    }
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  }
  s_s31_probe_end_cycle = last_sample_cycle;
  s_s31_probe_have_end_cycle = true;

  if (seamless_micro_gate) {
    for (uint32_t frame = 0u; frame < capture_chunks; ++frame) {
      volatile uint32_t *dst = (volatile uint32_t *)(
          (uintptr_t)slots[frame] + offsetof(iq_chunk_t, samples));
      uint32_t stage_off = frame * IQ_CHUNK_SAMPLE_WORDS;
      for (uint32_t word = 0u; word < IQ_CHUNK_SAMPLE_WORDS; ++word) {
        dst[word] = s_s31_gpio_diag_stage[stage_off + word];
      }
    }
  } else if (!lp_core_probe && !hp_tcm_probe) {
    for (uint32_t frame = 0u; frame < capture_chunks; ++frame) {
      volatile uint32_t *dst = (volatile uint32_t *)(
          (uintptr_t)slots[frame] + offsetof(iq_chunk_t, samples));
      uint32_t stage_off = frame * IQ_CHUNK_SAMPLE_WORDS;
      for (uint32_t word = 0u; word < IQ_CHUNK_SAMPLE_WORDS; ++word) {
        dst[word] = gpio_diag ? s_s31_gpio_diag_stage[stage_off + word]
                              : S31_STAGE_BASE[stage_off + word];
      }
    }
  }
  s31_finalize_usb_stage_batch(slots, capture_chunks, &diag);
  uint32_t actual_cycles =
      last_sample_cycle > first_sample_cycle
          ? (last_sample_cycle - first_sample_cycle) /
                (capture_words - 1u)
          : 0u;
  for (uint32_t frame = 0u; frame < capture_chunks; ++frame) {
    slots[frame]->iq.bank_timer_late_misses =
        gpio_diag ? diag_fix_read
                  : (seamless_micro_gate
                         ? diag.end_raw
                         : slots[frame]->iq.bank_timer_late_misses);
    slots[frame]->iq.bank_timer_write_ptr =
        gpio_diag ? diag_enable_read
                  : (seamless_micro_gate ? diag.start_raw : selector);
    slots[frame]->iq.producer_wake_write_ptr =
        0xd1000000u | (actual_cycles & 0xffffu);
  }
  if (!iq_usb_direct_commit(slots, capture_chunks)) {
    s_stream_dropped_chunks += capture_chunks;
    return;
  }

  uint32_t log_key = selector | (expert << 8u);
  /* Formatted logging can lazily allocate a libc mutex.  The USB direct slab
   * intentionally leaves very little internal heap, so never invoke it from
   * the GPIO diagnostic path after the stream buffers have been reserved. */
  if (!gpio_diag && s_s31_probe_logged_selector != log_key) {
    ESP_LOGI("iq_capture",
             "%s selector=%" PRIu32 " addr=%08" PRIxPTR
             " swap=%u raw=%08" PRIx32 "..%08" PRIx32
             " changes=%" PRIu32 "/%u cycles=%" PRIu32
             " late=%" PRIu32 " regs=%08" PRIx32 "/%08" PRIx32
             "/%08" PRIx32 "/%08" PRIx32 "/%08" PRIx32,
             seamless_micro_gate
                 ? "seamless_micro_gate"
                 : (lp_core_probe
                 ? "lp_core_probe"
                 : (hp_tcm_probe
                        ? "hp_tcm_probe"
                        : (gpio_diag
                               ? "gpio_diag"
                               : (datadump_mmio ? "datadump_mmio"
                                                : "modem_diag")))),
             selector,
             lp_core_probe ? (uintptr_t)LP_SYSTEM_REG_LP_PROBE_OUT_REG
                           : (gpio_diag ? (uintptr_t)&GPIO.in.val
                           : (datadump_mmio
                                  ? probe_addr
                                  : (uintptr_t)HP_SYSTEM_PROBE_OUT_REG)),
             swap ? 1u : 0u, raw_min, raw_max, changes,
             capture_words, actual_cycles, late_samples,
             reg32_read_addr(MODEM_WIDGETS_CLK_CONF_REG),
             reg32_read_addr(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG),
             reg32_read_addr(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG),
             reg32_read_addr(HP_SYSTEM_MODEM_DIAG_EN_REG),
             reg32p_read(&MODEM_WIFI_BB.BB_DIAG0));
    s_s31_probe_logged_selector = log_key;
  }
  s_s31_probe_source_chunk += capture_chunks;
  s_emit_chunk = s_s31_probe_source_chunk;
  s_source_chunk_index = s_emit_chunk;
}

#endif

/* PARLIO samples the live modem diagnostic bus into one circular GDMA
 * transaction: 16-bit native I/Q at 16 MHz on Ethernet, or the 8-bit real
 * lane at 4 MHz on USB.  There is deliberately no transaction boundary in
 * this path: descriptor callbacks only publish completed pieces of the same
 * permanently mounted ring. */
static bool s31_parlio_next_event(s31_parlio_event_t *event) {
  while (s_s31_parlio_running && iq_network_stream_armed()) {
    uint32_t head = __atomic_load_n(&s_s31_parlio_event_head,
                                    __ATOMIC_ACQUIRE);
    uint32_t backlog = head - s_s31_parlio_event_tail;
    if (backlog >= S31_PARLIO_SAFE_NODE_BACKLOG) {
      /* DMA is one descriptor away from reusing the oldest completed node.
       * Discard the complete backlog rather than race an overwrite. */
      s_s31_parlio_event_tail = head;
      ++s_s31_parlio_callback_overruns;
      ++s_s31_parlio_discontinuities;
      return false;
    }
    if (backlog != 0u) {
      uint32_t sequence = s_s31_parlio_event_tail;
      s31_parlio_event_t value =
          s_s31_parlio_events[sequence % S31_PARLIO_EVENT_RING_SIZE];
      __atomic_thread_fence(__ATOMIC_ACQUIRE);
      if (value.sequence != sequence || value.data == NULL ||
          value.bytes == 0u) {
        s_s31_parlio_event_tail = head;
        ++s_s31_parlio_discontinuities;
        return false;
      }
      /* The PARLIO ISR invalidates this node on the CPU that services GDMA,
       * but the producer task can run on the other S31 core.  Invalidate the
       * completed node again in the consumer context before memcpy.  Without
       * this, that core can retain individual 64-byte lines from the previous
       * lap of the circular buffer: coherent-tone tests then show phase jumps
       * every 32 complex samples and byte-identical frames exactly one
       * 14-frame DMA-ring lap apart.  PARLIO's 4092-byte nodes are not cache
       * aligned, while M2C msync deliberately rejects its UNALIGNED flag, so
       * cover the node with complete cache lines.  The receive buffer itself
       * is 64-byte aligned and padded to a complete line. */
      size_t cache_line = esp_cache_get_line_size_by_addr(value.data);
      if (cache_line != 0u) {
        uintptr_t begin = (uintptr_t)value.data & ~(cache_line - 1u);
        uintptr_t end = ((uintptr_t)value.data + value.bytes + cache_line - 1u) &
                        ~(cache_line - 1u);
        (void)esp_cache_msync((void *)begin, end - begin,
                              ESP_CACHE_MSYNC_FLAG_DIR_M2C);
      }
      /* Keep tail on this node until the producer has copied every byte. DMA
       * ownership must not be released merely because a pointer was handed
       * out: a preemption or a batch boundary could otherwise let GDMA wrap
       * and overwrite the node while it is still being read. */
      *event = value;
      return true;
    }
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
  }
  return false;
}

static void s31_parlio_release_pending_event(void) {
  if (s_s31_parlio_pending_event.data == NULL ||
      s_s31_parlio_pending_offset < s_s31_parlio_pending_event.bytes) {
    return;
  }
  s_s31_parlio_event_tail = s_s31_parlio_pending_event.sequence + 1u;
  s_s31_parlio_pending_event = (s31_parlio_event_t){0};
  s_s31_parlio_pending_offset = 0u;
}

static void s31_process_parlio_diag(const capture_config_t *config) {
  /* Expert sweeps can request a finite number of raw chunks through the
   * ordinary interval-duration field.  Stop producing at that exact batch
   * boundary so the Ethernet queue drains and control remains recoverable
   * even when an experimental rate exceeds the current transport ceiling. */
  uint32_t chunk_limit = config->trigger.trigger_config[1];
  if (chunk_limit != 0u && s_s31_probe_source_chunk >= chunk_limit) {
    s31_gpio_diag_stop();
    return;
  }
  s31_parlio_release_pending_event();
  if (s_s31_parlio_pending_offset < s_s31_parlio_pending_event.bytes) {
    uint32_t head = __atomic_load_n(&s_s31_parlio_event_head,
                                    __ATOMIC_ACQUIRE);
    if (head - s_s31_parlio_pending_event.sequence >=
        S31_PARLIO_SAFE_NODE_BACKLOG) {
      s_s31_parlio_pending_event = (s31_parlio_event_t){0};
      s_s31_parlio_pending_offset = 0u;
      s_s31_parlio_event_tail = head;
      ++s_s31_parlio_callback_overruns;
      ++s_s31_parlio_discontinuities;
    }
  }
  const bool host_real = s31_parlio_host_real_selected(config);
  const bool usb_iq4 = s31_parlio_usb_iq4_selected(config);
  const uint32_t desired_width = (host_real || usb_iq4) ? 8u : 16u;
  if (s_s31_parlio_unit != NULL &&
      s_s31_parlio_data_width != desired_width) {
    /* STREAM_START is the first point at which the transport owner and wire
     * format are known. Reconfigure the retained maximum-width unit while it
     * is stopped; rebuilding its GDMA objects after network startup is not
     * reliable in the small fragmented internal heap. */
    s31_gpio_diag_prepare(config);
  }
  const uint32_t output_chunks =
      host_real ? S31_CONTINUOUS_OUTPUT_CHUNKS : S31_PARLIO_BATCH_CHUNKS;
  const uint32_t frame_payload_bytes =
      host_real ? 4u * IQ_CHUNK_SAMPLE_WORDS
                : (usb_iq4 ? IQ_CHUNK_SAMPLE_WORDS
                           : 2u * IQ_CHUNK_SAMPLE_WORDS);
  stream_frame_t *slots[S31_PARLIO_BATCH_CHUNKS];
  if (s_s31_parlio_unit == NULL) {
    return;
  }
  /* Native USB can carry the already compact PARLIO IQ bytes straight from
   * the producer into DWC2's coherent slab.  The ordinary stream ring would
   * add a second PSRAM-to-PSRAM copy on core 0 and is the measured 16 MSa/s
   * bottleneck. Ethernet retains its zero-copy stream-ring path. */
  const bool direct_usb_compact =
      !host_real && iq_network_stream_owner() == IQ_STREAM_OWNER_USB &&
      (iq_usb_stream_format() == IQ_USB_FORMAT_INT8 || usb_iq4);
  const bool slots_reserved =
      direct_usb_compact
          ? (usb_iq4 ? iq_usb_direct_reserve_iq4(output_chunks, slots)
                     : iq_usb_direct_reserve_iq8(output_chunks, slots))
          : stream_ring_reserve_slots(output_chunks, slots);
  if (!slots_reserved) {
    return;
  }
  if (!s_s31_parlio_running) {
    s31_gpio_diag_start(config);
  }
  if (!s_s31_parlio_running) {
    if (direct_usb_compact) {
      iq_usb_direct_abort();
    } else {
      stream_ring_release_reserved(output_chunks);
    }
    return;
  }

  uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
  uint32_t rx_gain = agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M;
  uint32_t agc_state = (agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                       MODEM_WIFI_AGC_AGCRD3_STATE_S;
  for (uint32_t frame = 0u; frame < output_chunks; ++frame) {
    iq_chunk_report_meta_t meta = {
        .source_chunk_index = s_s31_probe_source_chunk + frame,
        .adc_decimation = adc_decimation_value(config),
        .sample_rate_hz = s31_parlio_capture_rate_hz(config),
        .center_freq_mhz = config->radio.rf_freq_hz / HZ_PER_MHZ,
        .rx_gain = rx_gain,
        .agc_state = agc_state,
        .dropped_chunks = s_stream_dropped_chunks,
        .bank_timer_late_misses = s_s31_parlio_callback_overruns,
        .bank_timer_write_ptr = s_s31_parlio_discontinuities,
        .producer_wake_write_ptr = 0u,
    };
    stream_ring_prepare_iq_chunk(slots[frame], &meta);
    if (host_real) {
      memcpy(slots[frame]->iq.magic, STREAM_FRAME_MAGIC_REAL8, 4u);
    } else if (usb_iq4) {
      memcpy(slots[frame]->iq.magic, STREAM_FRAME_MAGIC_IQ4, 4u);
    } else {
      slots[frame]->iq.magic[3] = '8';
    }
  }

  uint32_t batch_byte = 0u;
  const uint32_t batch_bytes = output_chunks * frame_payload_bytes;
  while (batch_byte < batch_bytes) {
    if (s_s31_parlio_pending_offset >=
        s_s31_parlio_pending_event.bytes) {
      if (!s31_parlio_next_event(&s_s31_parlio_pending_event)) {
        if (direct_usb_compact) {
          iq_usb_direct_abort();
        } else {
          stream_ring_release_reserved(output_chunks);
        }
        if (!iq_network_stream_armed()) {
          return;
        }
        s_stream_dropped_chunks += output_chunks;
        s_s31_probe_source_chunk += output_chunks;
        return;
      }
      s_s31_parlio_pending_offset = 0u;
    }
    uint32_t available = s_s31_parlio_pending_event.bytes -
                         s_s31_parlio_pending_offset;
    uint32_t event_take = batch_bytes - batch_byte;
    if (event_take > available) {
      event_take = available;
    }
    uint32_t copied = 0u;
    while (copied < event_take) {
      uint32_t frame = batch_byte / frame_payload_bytes;
      uint32_t byte = batch_byte % frame_payload_bytes;
      uint32_t take = event_take - copied;
      if (take > frame_payload_bytes - byte) {
        take = frame_payload_bytes - byte;
      }
      uint8_t *payload =
          (uint8_t *)slots[frame] + offsetof(iq_chunk_t, samples);
      memcpy(payload + byte, s_s31_parlio_pending_event.data +
                                 s_s31_parlio_pending_offset + copied,
             take);
      copied += take;
      batch_byte += take;
    }
    s_s31_parlio_pending_offset += event_take;
    s31_parlio_release_pending_event();
  }

  uint32_t backlog =
      __atomic_load_n(&s_s31_parlio_event_head, __ATOMIC_ACQUIRE) -
      s_s31_parlio_event_tail;
  for (uint32_t frame = 0u; frame < output_chunks; ++frame) {
    uint32_t *wire_crc = (uint32_t *)(
        (uint8_t *)slots[frame] + offsetof(iq_chunk_t, samples) +
        frame_payload_bytes);
    *wire_crc = 0u;
    slots[frame]->iq.dropped_chunks = s_stream_dropped_chunks;
    slots[frame]->iq.bank_timer_late_misses = s_s31_parlio_callback_overruns;
    slots[frame]->iq.bank_timer_write_ptr = s_s31_parlio_discontinuities;
    slots[frame]->iq.producer_wake_write_ptr = backlog;
  }
  if (direct_usb_compact) {
    if (!iq_usb_direct_commit(slots, output_chunks)) {
      s_stream_dropped_chunks += output_chunks;
    }
  } else {
    stream_ring_commit_reserved(output_chunks);
  }
  s_s31_probe_source_chunk += output_chunks;
  s_emit_chunk = s_s31_probe_source_chunk;
  s_source_chunk_index = s_emit_chunk;
}

static uint32_t IRAM_ATTR s31_ring_guard_words(const trigger_plan_t *plan) {
  if (plan->mode == IQ_TRIGGER_MODE_INTERVAL && plan->interval_chunks == 1u &&
      plan->interval_duration_chunks != 0u) {
    return S31_LIVE_COPY_GUARD_WORDS;
  }
  return S31_RING_GUARD_WORDS;
}

static bool s31_pulse_tcm_before_copy(const capture_config_t *config) {
  return (config->iq_engine.adc_source_sel &
          S31_ADC_SOURCE_PULSE_TCM_BEFORE_COPY) != 0u;
}

static uint32_t s31_pulse_tcm_dwell(const capture_config_t *config) {
  if ((config->iq_engine.adc_source_sel & S31_ADC_SOURCE_LONG_GATE_DWELL) !=
      0u) {
    return 8u * DUMP_BANK_WORDS;
  }
  uint32_t hardware_decimation_field =
      (config->iq_engine.adc_source_sel & S31_ADC_SOURCE_HW_DECIM_M) >>
      S31_ADC_SOURCE_HW_DECIM_S;
  if (hardware_decimation_field == 7u) {
    uint32_t dwell = 32768u;
    if ((config->iq_engine.adc_source_sel & S31_ADC_SOURCE_FIELD7_DWELL_X2) !=
        0u) {
      dwell *= 2u;
    }
    if ((config->iq_engine.adc_source_sel & S31_ADC_SOURCE_FIELD7_DWELL_X4) !=
        0u) {
      dwell *= 4u;
    }
    return dwell;
  }
  uint32_t dwell_sel =
      (config->iq_engine.adc_source_sel & S31_ADC_SOURCE_PULSE_DWELL_M) >>
      S31_ADC_SOURCE_PULSE_DWELL_S;
  /* Amortize each settled snapshot over a safe output batch. The encoded
   * 1,024-cycle dwell leaves stale regions. A 2,048-cycle dwell substantially
   * improves OFDM coherence through decimation 8, while decimations 9 and 10
   * use a conservative 1,280-cycle dwell to preserve every source chunk.
   * Explicit diagnostic source values retain their encoded power-of-two
   * dwell. */
  if (config->iq_engine.adc_source_sel == ADC_DUMP_SOURCE_CONTINUOUS_RX_IQ) {
    return s31_software_decimation(config) <= 8u ? 2048u : 1280u;
  }
  return 64u << dwell_sel;
}

static uint32_t s31_pulse_tcm_mask(const capture_config_t *config) {
  uint32_t mask =
      (config->iq_engine.adc_source_sel & S31_ADC_SOURCE_TCM_MASK_M) >>
      S31_ADC_SOURCE_TCM_MASK_S;
  return (mask == 0u ? 0xffu : mask) << 24;
}

static int32_t s31_live_copy_offset_chunks(const capture_config_t *config) {
  (void)config;
  return 0;
}

static uint32_t s31_output_chunk_words(const capture_config_t *config) {
  return IQ_CHUNK_SAMPLE_WORDS * s31_software_decimation(config);
}

static void IRAM_ATTR s31_msync_ring_span(uint32_t start_word,
                                          uint32_t word_count) {
  uint32_t off = dump_ring_mod(start_word);
  uint32_t first = DUMP_BANK_WORDS - off;
  if (first > word_count) {
    first = word_count;
  }
  (void)esp_cache_msync((void *)&DUMP_BASE[off], first * sizeof(uint32_t),
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  if (word_count > first) {
    (void)esp_cache_msync((void *)DUMP_BASE,
                          (word_count - first) * sizeof(uint32_t),
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  }
}

static void s31_pulse_tcm_dump_gate(const capture_config_t *config) {
  if ((config->iq_engine.adc_source_sel & S31_ADC_SOURCE_TCM_ALWAYS_ON) != 0u) {
    return;
  }
  /* TCM dump access is visible to both HP cores.  Park the peer in the IPC
   * high-priority ISR and mask local interrupts so neither scheduler can touch
   * TCM while the modem gate owns it. */
  esp_ipc_isr_stall_other_cpu();
  taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                   s31_pulse_tcm_mask(config));
  __asm__ __volatile__("fence" ::: "memory");
  uint32_t dwell = s31_pulse_tcm_dwell(config);
  for (volatile uint32_t i = 0u; i < dwell; ++i) {
    __asm__ __volatile__("nop");
  }
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  __asm__ __volatile__("fence" ::: "memory");
  taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  esp_ipc_isr_release_other_cpu();
}

static bool s31_pipelined_stage_enabled(const capture_config_t *config) {
  return (config->iq_engine.adc_source_sel &
          S31_ADC_SOURCE_PIPELINED_STAGE) != 0u;
}

/* Copy two already-completed dump words during ownership slots synchronized
 * to a writer-pointer edge.  At 4 MS/s the 300 MHz HP core has 75 cycles
 * between samples.  If each closed interval is shorter than that and begins
 * immediately after an edge, the modem never reaches a write while its TCM
 * route is disconnected.  Pointer advance is measured across every slot;
 * Pluto phase continuity remains the independent end-to-end oracle. */
static void IRAM_ATTR s31_micro_gate_capture(
    volatile uint32_t *dst, uint32_t start_raw, uint32_t word_count,
    uint32_t gate_mask, uint32_t *pointer_advances,
    uint32_t *max_gate_cycles) {
  const uint32_t burst_words = 2u;
  uint32_t advances = 0u;
  uint32_t max_cycles = 0u;

  for (uint32_t copied = 0u; copied < word_count; copied += burst_words) {
    uint32_t source = start_raw + copied;
    if (source >= DUMP_BANK_WORDS) {
      source -= DUMP_BANK_WORDS;
    }
    volatile uint32_t *src = &DUMP_BASE[source];
    uint32_t target = copied + burst_words;
    uint32_t edge_raw;
    do {
      edge_raw = adc_dump_store_addr_raw();
      uint32_t ready = edge_raw >= start_raw
                           ? edge_raw - start_raw
                           : DUMP_BANK_WORDS - start_raw + edge_raw;
      if (ready >= target) {
        break;
      }
    } while (true);

    /* Even when a backlog exists, wait for a fresh edge so the gate closes at
     * the beginning rather than the end of a sample interval. */
    uint32_t before_raw;
    do {
      before_raw = adc_dump_store_addr_raw();
    } while (before_raw == edge_raw);

    uint32_t close_cycle = esp_cpu_get_cycle_count();
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    __asm__ __volatile__("fence rw, rw" ::: "memory");

    /* start_raw and the ring length are two-word aligned, so a burst never
     * straddles the physical end of the aperture. */
    uint32_t v0 = src[0];
    uint32_t v1 = src[1];
    dst[copied + 0u] = v0;
    dst[copied + 1u] = v1;

    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, gate_mask);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    uint32_t gate_cycles = esp_cpu_get_cycle_count() - close_cycle;
    if (gate_cycles > max_cycles) {
      max_cycles = gate_cycles;
    }
    uint32_t after_raw = adc_dump_store_addr_raw();
    advances += after_raw >= before_raw
                    ? after_raw - before_raw
                    : DUMP_BANK_WORDS - before_raw + after_raw;
  }

  *pointer_advances = advances;
  *max_gate_cycles = max_cycles;
}

/* The normal S31 snapshot path writes every frame directly to PSRAM after the
 * dump switch closes.  At 8 MS/s that keeps the writer disconnected for about
 * seven chunks per 16-chunk batch.  This diagnostic backend pipelines through
 * the reserved 64 KiB TCM guard instead:
 *
 *   writer open:  move the previous staging block to PSRAM while acquiring;
 *   writer closed: copy the new 64 KiB block TCM-to-TCM;
 *
 * The entire transaction executes from flash with a PSRAM producer stack and
 * both HP cores quiesced.  Volatile word loops deliberately prevent GCC from
 * substituting an internal-RAM memcpy while the fabric switch is open. */
static uint32_t __attribute__((noinline)) s31_stage_capture_transaction(
    const capture_config_t *config, stream_frame_t **slots,
    uint32_t requested_frames, s31_stage_batch_diag_t *diag) {
  uint32_t requested_batches = requested_frames / S31_STAGE_CHUNKS;
  if (requested_batches == 0u) {
    return 0u;
  }

  uint32_t gate_mask = s31_pulse_tcm_mask(config);
  const bool micro_gate =
      config->rx_filter.rx_filter_override == S31_MICRO_GATE_OVERRIDE;
  const bool ring_cursor =
      config->rx_filter.rx_filter_override == S31_RING_CURSOR_OVERRIDE;
  bool stage_valid = s_s31_stage_valid;
  uint32_t stage_source_chunk = s_s31_stage_source_chunk;
  uint32_t prev_end_raw = s_s31_stage_prev_end_raw;
  bool have_prev_end = s_s31_stage_have_prev_end;
  uint32_t produced_batches = 0u;

  /* The live dump aperture remaps TCM as observed by both HP cores.  Keep the
   * peer parked for every transport; letting Ethernet/lwIP run on core 0 while
   * the gate is open causes arbitrary ROM/lwIP load faults.  The Ethernet
   * caller deliberately limits this to one 14-frame acquisition window so
   * the GMAC interrupt blackout remains bounded. */
  esp_ipc_isr_stall_other_cpu();
  taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
  while (produced_batches < requested_batches) {
    uint32_t output_batch = produced_batches;
    uint32_t output_source_chunk = stage_source_chunk;

    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, gate_mask);
    __asm__ __volatile__("fence" ::: "memory");
    uint32_t start_raw;
    uint32_t writer_raw;
    if (ring_cursor && have_prev_end) {
      /* Keep consuming immediately after the preceding block. The writer
       * pointer proves availability; it must never redefine the consumer. */
      start_raw = prev_end_raw;
      writer_raw = dump_ring_mod(adc_dump_store_addr_raw());
    } else {
      /* The initial cursor is aligned so both pieces of a possible PIE ring
       * wrap remain valid operands. */
      do {
        writer_raw = dump_ring_mod(adc_dump_store_addr_raw());
        start_raw = writer_raw;
      } while ((start_raw & 3u) != 0u);
    }

    if (ring_cursor) {
      if (!have_prev_end) {
        stage_source_chunk = 0u;
      }
    } else {
      uint32_t gap_words =
          have_prev_end ? dump_ring_delta(start_raw, prev_end_raw) : 0u;
      uint32_t gap_chunks =
          (gap_words + IQ_CHUNK_SAMPLE_WORDS - 1u) / IQ_CHUNK_SAMPLE_WORDS;
      if (have_prev_end) {
        s_stream_dropped_chunks += gap_chunks;
        stage_source_chunk += S31_STAGE_CHUNKS + gap_chunks;
      } else {
        stage_source_chunk = 0u;
      }
    }

    if (stage_valid) {
      diag[output_batch].source_chunk = output_source_chunk;
      diag[output_batch].start_raw = prev_end_raw;
      diag[output_batch].end_raw = start_raw;
#if CONFIG_ESP_SDR_TRANSPORT_USB
      /* Header stores target the narrow write-through USB slab. Hide them
       * behind acquisition: start_raw was already sampled, so the writer's
       * progress during metadata preparation remains part of this block. */
      s31_prepare_stage_frames(
          config, &slots[output_batch * S31_STAGE_CHUNKS],
          S31_STAGE_CHUNKS, &diag[output_batch]);
#endif
      for (uint32_t chunk = 0u; chunk < S31_STAGE_CHUNKS; ++chunk) {
        stream_frame_t *slot =
            slots[output_batch * S31_STAGE_CHUNKS + chunk];
        volatile uint32_t *dst = (volatile uint32_t *)(
            (uintptr_t)slot + offsetof(iq_chunk_t, samples));
        uint32_t stage_off = chunk * IQ_CHUNK_SAMPLE_WORDS;
        for (uint32_t word = 0u; word < IQ_CHUNK_SAMPLE_WORDS; ++word) {
          dst[word] = S31_STAGE_BASE[stage_off + word];
        }
      }
#if CONFIG_ESP_SDR_TRANSPORT_USB
      s31_finalize_usb_stage_batch(
          &slots[output_batch * S31_STAGE_CHUNKS], S31_STAGE_CHUNKS,
          &diag[output_batch]);
#endif
    }

    uint32_t end_raw;
    if (micro_gate) {
      uint32_t pointer_advances;
      uint32_t max_gate_cycles;
      s31_micro_gate_capture(S31_STAGE_BASE, start_raw, S31_STAGE_WORDS,
                             gate_mask, &pointer_advances, &max_gate_cycles);
      end_raw = dump_ring_mod(start_raw + S31_STAGE_WORDS);
      s_s31_micro_gate_advances += pointer_advances;
      if (max_gate_cycles > s_s31_micro_gate_max_cycles) {
        s_s31_micro_gate_max_cycles = max_gate_cycles;
      }
      ++s_s31_micro_gate_batches;
      /* Runtime and transport are not yet safe under writer ownership.  Close
       * once per transaction here; the next iteration measures this remaining
       * coarse service interval separately as its normal inter-stage gap. */
      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
      __asm__ __volatile__("fence rw, rw" ::: "memory");
    } else if (ring_cursor) {
      uint32_t sample_cycles = adc_dump_sample_cycles(config);
      uint32_t cpu_cycles_per_sample =
          sample_cycles * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ /
          (ADC_DUMP_CLOCK_HZ / 1000000u);
      if (cpu_cycles_per_sample == 0u) {
        cpu_cycles_per_sample = 1u;
      }

      /* A modulo pointer cannot distinguish empty from producer-lapped.
       * Independently bound writer progress with the CPU cycle counter. Any
       * lap is reported and resynchronized, and must remain zero in sustained
       * operation. */
      uint32_t writer_cycle = esp_cpu_get_cycle_count();
      if (s_s31_cursor_have_writer_cycle) {
        uint32_t elapsed_samples =
            (writer_cycle - s_s31_cursor_last_writer_cycle) /
            cpu_cycles_per_sample;
        if (s_s31_cursor_last_backlog + elapsed_samples >= DUMP_BANK_WORDS) {
          ++s_s31_cursor_overruns;
          uint32_t lost_words =
              s_s31_cursor_last_backlog + elapsed_samples -
              (DUMP_BANK_WORDS - 1u);
          uint32_t lost_chunks =
              (lost_words + IQ_CHUNK_SAMPLE_WORDS - 1u) /
              IQ_CHUNK_SAMPLE_WORDS;
          s_stream_dropped_chunks += lost_chunks;
          stage_source_chunk += lost_chunks;
          do {
            writer_raw = dump_ring_mod(adc_dump_store_addr_raw());
            start_raw = writer_raw;
          } while ((start_raw & 3u) != 0u);
        }
      }

      do {
        writer_raw = dump_ring_mod(adc_dump_store_addr_raw());
      } while (dump_ring_delta(writer_raw, start_raw) < S31_STAGE_WORDS);

      uint32_t end_cursor = dump_ring_mod(start_raw + S31_STAGE_WORDS);
      uint32_t backlog = dump_ring_delta(writer_raw, end_cursor);
      if (backlog < s_s31_cursor_min_backlog) {
        s_s31_cursor_min_backlog = backlog;
      }
      if (backlog > s_s31_cursor_max_backlog) {
        s_s31_cursor_max_backlog = backlog;
      }
      ++s_s31_cursor_batches;
      s_s31_cursor_last_writer_cycle = esp_cpu_get_cycle_count();
      s_s31_cursor_last_backlog = backlog;
      s_s31_cursor_have_writer_cycle = true;
      end_raw = end_cursor;

      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
      __asm__ __volatile__("fence" ::: "memory");
    } else {
      do {
        end_raw = dump_ring_mod(adc_dump_store_addr_raw());
      } while (dump_ring_delta(end_raw, start_raw) < S31_STAGE_WORDS);

      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
      __asm__ __volatile__("fence" ::: "memory");
    }

    if (stage_valid) {
      ++produced_batches;
    }

    uint32_t start_word = dump_ring_mod(start_raw);
    /* The live aperture and staging guard are uncached internal TCM.  Cache
     * synchronization here only lengthens the writer-off interval; the dump
     * fabric handoff plus the full memory fence above makes the completed
     * snapshot directly visible to PIE. */
    if (!micro_gate) {
      uint32_t first_words = DUMP_BANK_WORDS - start_word;
      if (first_words > S31_STAGE_WORDS) {
        first_words = S31_STAGE_WORDS;
      }
      s31_pie_memcpy_aligned((void *)(uintptr_t)S31_STAGE_BASE,
                             (const void *)(uintptr_t)&DUMP_BASE[start_word],
                             first_words * sizeof(uint32_t));
      if (first_words < S31_STAGE_WORDS) {
        s31_pie_memcpy_aligned(
            (void *)(uintptr_t)&S31_STAGE_BASE[first_words],
            (const void *)(uintptr_t)DUMP_BASE,
            (S31_STAGE_WORDS - first_words) * sizeof(uint32_t));
      }
    }
    stage_valid = true;
    if (ring_cursor && have_prev_end) {
      stage_source_chunk += S31_STAGE_CHUNKS;
    }
    have_prev_end = true;
    prev_end_raw = end_raw;
  }
  taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  esp_ipc_isr_release_other_cpu();

  s_s31_stage_valid = stage_valid;
  s_s31_stage_source_chunk = stage_source_chunk;
  s_s31_stage_prev_end_raw = prev_end_raw;
  s_s31_stage_have_prev_end = have_prev_end;
  return produced_batches * S31_STAGE_CHUNKS;
}

static void IRAM_ATTR process_s31_pipelined_stage(
    const capture_config_t *config) {
#if CONFIG_ESP_SDR_TRANSPORT_USB
  /* One 14-frame transfer lasts less than the following 14-frame acquisition
   * at the sustainable full-precision USB rate. Ping-pong whole transfers;
   * asking for two at once prevents the completion ISR from rearming DWC2
   * while core 0 is parked and creates a multi-millisecond writer-off gap.
   * Run several transactions per producer turn so scheduler/config-service
   * latency is paid only once per group. */
  for (uint32_t round = 0u; round < 4u; ++round) {
    uint32_t n = S31_STAGE_CHUNKS;
    stream_frame_t *slots[S31_STAGE_CHUNKS];
    if (!iq_usb_direct_reserve(n, slots)) {
      return;
    }
    s31_stage_batch_diag_t diag[1u];
    uint32_t produced =
        s31_stage_capture_transaction(config, slots, n, diag);
    if (produced != n) {
      iq_usb_direct_abort();
      return;
    }
    if (config->rx_filter.rx_filter_override == S31_MICRO_GATE_OVERRIDE &&
        (s_s31_micro_gate_batches & 63u) == 0u) {
      ESP_LOGI("iq_capture",
               "micro_gate batches=%" PRIu32 " pointer_advances=%" PRIu32
               " max_closed_cycles=%" PRIu32,
               s_s31_micro_gate_batches, s_s31_micro_gate_advances,
               s_s31_micro_gate_max_cycles);
    }
    if (config->rx_filter.rx_filter_override == S31_RING_CURSOR_OVERRIDE &&
        (s_s31_cursor_batches & 63u) == 0u) {
      ESP_LOGI("iq_capture",
               "ring_cursor batches=%" PRIu32 " backlog=%" PRIu32
               "..%" PRIu32 " overruns=%" PRIu32,
               s_s31_cursor_batches, s_s31_cursor_min_backlog,
               s_s31_cursor_max_backlog, s_s31_cursor_overruns);
    }

    /* Feed every transaction so the DCO servo converges at its intended
     * cadence. Its internal 4-ms limiter makes non-update calls inexpensive. */
    const volatile uint32_t *servo_words =
        (const volatile uint32_t *)((uintptr_t)slots[produced - 1u] +
                                    offsetof(iq_chunk_t, samples));
    dcoc_feed_chunk(servo_words);
    if (!iq_usb_direct_commit(slots, n)) {
      s_stream_dropped_chunks += n;
      return;
    }
    s_emit_chunk = diag[0].source_chunk + S31_STAGE_CHUNKS;
    s_source_chunk_index = s_emit_chunk;
    if (!iq_network_stream_armed() || config_apply_in_progress()) {
      return;
    }
  }
  return;
#else
  uint32_t available = stream_ring_available_slots();
  /* Keep the peer core and Ethernet ISR parked for only one capture window.
   * Longer transactions starve the GMAC/lwIP control path even when IQC8
   * reduces the downstream wire rate. */
  uint32_t n = available > S31_STAGE_CHUNKS ? S31_STAGE_CHUNKS : available;
  n -= n % S31_STAGE_CHUNKS;
  if (n == 0u) {
    return;
  }

  stream_frame_t *slots[IQ_STREAM_RING_CHUNKS];
  if (!stream_ring_reserve_slots(n, slots)) {
    return;
  }
  s31_stage_batch_diag_t diag[IQ_STREAM_RING_CHUNKS / S31_STAGE_CHUNKS];
  uint32_t produced =
      s31_stage_capture_transaction(config, slots, n, diag);
  if (produced == 0u) {
    stream_ring_release_reserved(n);
    return;
  }
  if (produced < n) {
    stream_ring_release_reserved(n - produced);
    n = produced;
  }

  /* The staged path bypasses fill_and_push_chunk(), which normally samples
   * each emitted chunk for the slow DCOC servo.  Service it only after
   * the TCM fabric has been returned to the CPUs: dcoc_feed() may read the
   * timer and occasionally rewrite/re-latch gain RAM, neither of which is
   * safe while the writer-owned aperture and interrupt gate are active.
   * One representative completed chunk per transaction is sufficient; the
   * servo has its own 4-ms rate limiter. */
  const volatile uint32_t *servo_words =
      (const volatile uint32_t *)((uintptr_t)slots[produced - 1u] +
                                  offsetof(iq_chunk_t, samples));
  dcoc_feed_chunk(servo_words);

  s31_prepare_stage_frames(config, slots, n, diag);
  stream_ring_commit_reserved(n);
  s_emit_chunk = diag[(n - 1u) / S31_STAGE_CHUNKS].source_chunk +
                 S31_STAGE_CHUNKS;
  s_source_chunk_index = s_emit_chunk;
  /* A full-duty producer otherwise re-enters the dual-core TCM stall before
   * lwIP can drain control traffic. Give core 0 one scheduler tick between
   * snapshots; the next source index honestly accounts for this interval. */
  vTaskDelay(pdMS_TO_TICKS(1));
#endif
}

static const uint32_t *IRAM_ATTR s31_chunk_source(
    const capture_config_t *config, uint32_t c, uint64_t abs_write,
    bool refresh_dump_window) {
  uint32_t decimation = s31_software_decimation(config);
  uint32_t output_words = s31_output_chunk_words(config);
  uint32_t chunks_per_window = DUMP_BANK_WORDS / IQ_CHUNK_SAMPLE_WORDS;
  uint64_t start_abs = (uint64_t)c * output_words;
  uint32_t start_word = (uint32_t)(start_abs % DUMP_BANK_WORDS);
  uint32_t physical_chunk =
      (start_word / IQ_CHUNK_SAMPLE_WORDS) % chunks_per_window;
  uint64_t abs_chunk = abs_write / output_words;
  int32_t physical_delta =
      (int32_t)physical_chunk - (int32_t)(abs_chunk % chunks_per_window);
  if (physical_delta > 0) {
    physical_delta -= (int32_t)chunks_per_window;
  }
  int64_t source_abs = (int64_t)abs_chunk + physical_delta;
  uint32_t source_chunk = source_abs > 0 ? (uint32_t)source_abs : 0u;
  s_s31_copy_ctrl_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL);
  s_s31_copy_mode_before_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_MODE);
  if (refresh_dump_window && s31_pulse_tcm_before_copy(config)) {
    s31_pulse_tcm_dump_gate(config);
  }
  s_s31_copy_mode_after_diag = adc_dump_store_addr_raw();
  s_s31_copy_physical_chunk = physical_chunk;
  s_s31_copy_source_chunk = source_chunk;
  if (decimation == 1u) {
    uint32_t off = dump_ring_mod(start_word);
    (void)esp_cache_msync((void *)&DUMP_BASE[off],
                          IQ_CHUNK_SAMPLE_WORDS * sizeof(uint32_t),
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    return (const uint32_t *)&DUMP_BASE[off];
  }
  s31_msync_ring_span(start_word, output_words);
  uint32_t source_index = dump_ring_mod(start_word);
  for (uint32_t i = 0u; i < IQ_CHUNK_SAMPLE_WORDS; ++i) {
    s_s31_decim_buf[i] = DUMP_BASE[source_index];
    source_index += decimation;
    if (source_index >= DUMP_BANK_WORDS)
      source_index -= DUMP_BANK_WORDS;
  }
  return s_s31_decim_buf;
}

static const uint32_t *IRAM_ATTR
s31_exact_chunk_source(const capture_config_t *config, uint32_t source_chunk) {
  uint32_t decimation = s31_software_decimation(config);
  uint32_t output_words = s31_output_chunk_words(config);
  uint32_t chunks_per_window = DUMP_BANK_WORDS / IQ_CHUNK_SAMPLE_WORDS;
  uint64_t start_abs = (uint64_t)source_chunk * output_words;
  uint32_t start_word = (uint32_t)(start_abs % DUMP_BANK_WORDS);
  uint32_t physical_chunk =
      (start_word / IQ_CHUNK_SAMPLE_WORDS) % chunks_per_window;
  s_s31_copy_ctrl_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL);
  s_s31_copy_mode_before_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_MODE);
  if (s31_pulse_tcm_before_copy(config)) {
    s31_pulse_tcm_dump_gate(config);
  }
  s_s31_copy_mode_after_diag = adc_dump_store_addr_raw();
  s_s31_copy_physical_chunk = physical_chunk;
  s_s31_copy_source_chunk = source_chunk;
  if (decimation == 1u) {
    uint32_t off = dump_ring_mod(start_word);
    (void)esp_cache_msync((void *)&DUMP_BASE[off],
                          IQ_CHUNK_SAMPLE_WORDS * sizeof(uint32_t),
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    return (const uint32_t *)&DUMP_BASE[off];
  }
  s31_msync_ring_span(start_word, output_words);
  uint32_t source_index = dump_ring_mod(start_word);
  for (uint32_t i = 0u; i < IQ_CHUNK_SAMPLE_WORDS; ++i) {
    s_s31_decim_buf[i] = DUMP_BASE[source_index];
    source_index += decimation;
    if (source_index >= DUMP_BANK_WORDS)
      source_index -= DUMP_BANK_WORDS;
  }
  return s_s31_decim_buf;
}

static void IRAM_ATTR s31_chunk_copy_done(uint32_t c) { (void)c; }

static uint64_t IRAM_ATTR
s31_current_abs_write(const capture_config_t *config) {
  int64_t now_us = esp_timer_get_time();
  uint32_t raw_sa = adc_dump_store_addr_raw();
  uint32_t sa = dump_ring_mod(adc_dump_store_addr_from_raw(raw_sa));
  uint32_t delta_mod = dump_ring_delta(sa, s_isr_prev_sa);
  uint32_t delta = delta_mod;
  uint32_t sample_cycles = adc_dump_write_sample_cycles(config);
  if (s_s31_abs_track_us != 0 && sample_cycles != 0u) {
    uint64_t elapsed_us = (uint64_t)(now_us - s_s31_abs_track_us);
    uint64_t expected_delta = (elapsed_us * (uint64_t)ADC_DUMP_CLOCK_HZ) /
                              ((uint64_t)sample_cycles * 1000000ull);
    if (expected_delta > (DUMP_BANK_WORDS / 2u)) {
      uint64_t wraps = 0u;
      if (expected_delta > delta_mod) {
        wraps = (expected_delta - delta_mod + (DUMP_BANK_WORDS / 2u)) /
                DUMP_BANK_WORDS;
      }
      uint64_t delta64 = (uint64_t)delta_mod + wraps * DUMP_BANK_WORDS;
      delta = delta64 > UINT32_MAX ? UINT32_MAX : (uint32_t)delta64;
    }
  }
  s_s31_abs_write_accum += delta;
  s_isr_wrap_count = (uint32_t)(s_s31_abs_write_accum / DUMP_BANK_WORDS);
  s_isr_prev_sa = sa;
  s_s31_abs_track_us = now_us;
  s_dbg_lo = raw_sa;
  uint64_t abs_write = s_s31_abs_write_accum;
  s_s31_diag_abs_write = abs_write;
  return abs_write;
}

static bool IRAM_ATTR s31_wait_for_abs_write(const capture_config_t *config,
                                             uint64_t target_abs,
                                             uint64_t *abs_write) {
  int64_t wait_start_us = esp_timer_get_time();
  uint32_t sample_cycles = adc_dump_write_sample_cycles(config);
  while (*abs_write < target_abs) {
    uint64_t remaining_words64 = target_abs - *abs_write;
    uint32_t remaining_words = remaining_words64 > UINT32_MAX
                                   ? UINT32_MAX
                                   : (uint32_t)remaining_words64;
    uint32_t remaining_us =
        sample_cycles != 0u
            ? (remaining_words * sample_cycles) /
                  (ADC_DUMP_CLOCK_HZ / 1000000u)
            : 0u;
    if (remaining_us > S31_POLL_WAKE_MARGIN_US + 1000u) {
      vTaskDelay(
          pdMS_TO_TICKS((remaining_us - S31_POLL_WAKE_MARGIN_US) / 1000u));
    } else if (remaining_us > 80u) {
      esp_rom_delay_us(remaining_us - S31_POLL_CLOSE_US);
    } else {
      __asm__ __volatile__("nop");
    }
    *abs_write = s31_current_abs_write(config);
    if ((uint32_t)(esp_timer_get_time() - wait_start_us) > 100000u) {
      return false;
    }
  }
  return true;
}

static void IRAM_ATTR process_s31_live_pulse_interval(
    const capture_config_t *config, const trigger_plan_t *plan,
    uint64_t abs_write) {
  uint32_t output_words = s31_output_chunk_words(config);
  if (plan->interval_chunks == 1u &&
      plan->interval_duration_chunks != 0u) {
    if (s31_pipelined_stage_enabled(config) &&
        s31_software_decimation(config) == 1u && !stream_output_int8()) {
      process_s31_pipelined_stage(config);
      return;
    }
    if (abs_write <= S31_LIVE_COPY_GUARD_WORDS) {
      return;
    }
    uint32_t ready_chunk = (uint32_t)(
        (abs_write - S31_LIVE_COPY_GUARD_WORDS) / output_words);
    if (!s_s31_ring_started) {
      s_emit_chunk = ready_chunk > 0u ? ready_chunk - 1u : 0u;
      s_s31_ring_started = true;
    }
    uint64_t low_abs = abs_write > DUMP_BANK_WORDS
                           ? abs_write - DUMP_BANK_WORDS + output_words
                           : 0u;
    uint32_t low_chunk =
        (uint32_t)((low_abs + output_words - 1u) / output_words);
    if (s_emit_chunk < low_chunk) {
      s_stream_dropped_chunks += low_chunk - s_emit_chunk;
      s_emit_chunk = low_chunk;
    }
    if (ready_chunk <= s_emit_chunk) {
      s_source_chunk_index = s_emit_chunk;
      return;
    }
    uint32_t n = ready_chunk - s_emit_chunk;
    uint32_t safe_chunks =
        (DUMP_BANK_WORDS - S31_LIVE_COPY_GUARD_WORDS) / output_words;
    uint32_t desired_batch = safe_chunks > 3u ? safe_chunks - 3u : 1u;
    if (n < desired_batch) {
      s_source_chunk_index = s_emit_chunk;
      return;
    }
    if (n > S31_RING_MAX_CHUNKS_PER_POLL) {
      n = S31_RING_MAX_CHUNKS_PER_POLL;
    }
    uint32_t available = stream_ring_available_slots();
    if (n > available) {
      n = available;
    }
    if (n == 0u) {
      return;
    }
    stream_frame_t *slots[S31_RING_MAX_CHUNKS_PER_POLL];
    if (!stream_ring_reserve_slots(n, slots)) {
      return;
    }
    uint32_t sample_cycles = adc_dump_sample_cycles(config);
    uint32_t sample_rate_hz =
        sample_cycles != 0u ? ADC_DUMP_CLOCK_HZ / sample_cycles : 0u;
    uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
    uint32_t rx_gain = agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M;
    uint32_t agc_state = (agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                         MODEM_WIFI_AGC_AGCRD3_STATE_S;
    s31_pulse_tcm_dump_gate(config);
    s_s31_copy_ctrl_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL);
    s_s31_copy_mode_before_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_MODE);
    s_s31_copy_mode_after_diag = adc_dump_store_addr_raw();
    uint32_t decimation = s31_software_decimation(config);
    for (uint32_t i = 0u; i < n; ++i) {
      uint32_t source_chunk = s_emit_chunk + i;
      uint32_t start_word = (uint32_t)(
          ((uint64_t)source_chunk * output_words) % DUMP_BANK_WORDS);
      fill_and_push_chunk_strided(config, source_chunk, sample_rate_hz,
                                  rx_gain, agc_state, slots[i], start_word,
                                  decimation);
    }
    stream_ring_commit_reserved(n);
    s_emit_chunk += n;
    s_source_chunk_index = s_emit_chunk;
    return;
  }
  uint32_t abs_chunk = (uint32_t)(abs_write / output_words);
  if (!s_s31_ring_started) {
    s_emit_chunk = trigger_interval_next_selected(plan, abs_chunk);
    s_s31_ring_started = true;
  }
  if (abs_chunk < s_emit_chunk) {
    return;
  }

  int32_t offset = s31_live_copy_offset_chunks(config);
  int64_t copy_chunk_i64 = (int64_t)s_emit_chunk - offset;
  if (copy_chunk_i64 < 0) {
    copy_chunk_i64 = 0;
  }
  uint32_t copy_chunk = (uint32_t)copy_chunk_i64;
  uint64_t target_ready_abs =
      ((uint64_t)copy_chunk + 1u) * output_words +
      S31_LIVE_COPY_GUARD_WORDS;
  if (!s31_wait_for_abs_write(config, target_ready_abs, &abs_write)) {
    ++s_stream_dropped_chunks;
    s_emit_chunk = trigger_interval_next_selected(plan, abs_chunk + 1u);
    s_source_chunk_index = s_emit_chunk;
    return;
  }

  uint64_t stale_words =
      abs_write - ((uint64_t)s_emit_chunk * output_words);
  if (stale_words >= DUMP_BANK_WORDS - output_words) {
    uint32_t new_emit_chunk = abs_write / output_words;
    s_stream_dropped_chunks +=
        trigger_interval_selected_count(plan, s_emit_chunk, new_emit_chunk);
    s_emit_chunk = trigger_interval_next_selected(plan, new_emit_chunk);
    s_source_chunk_index = s_emit_chunk;
    return;
  }

  stream_frame_t *slots[1];
  if (!stream_ring_reserve_slots(1u, slots)) {
    ++s_stream_dropped_chunks;
    s_emit_chunk = trigger_interval_next_selected(plan, abs_chunk + 1u);
    s_source_chunk_index = s_emit_chunk;
    return;
  }

  uint32_t sample_cycles = adc_dump_sample_cycles(config);
  uint32_t sample_rate_hz =
      sample_cycles != 0u ? ADC_DUMP_CLOCK_HZ / sample_cycles : 0u;
  uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
  uint32_t rx_gain = agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M;
  uint32_t agc_state = (agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                       MODEM_WIFI_AGC_AGCRD3_STATE_S;
  uint32_t logical_chunk = s_emit_chunk;
  const uint32_t *words =
      s31_chunk_source(config, copy_chunk, abs_write, true);
  fill_and_push_chunk(config, words, logical_chunk, sample_rate_hz, rx_gain,
                      agc_state, slots[0]);
  stream_ring_commit_reserved(1u);

  s_emit_chunk = trigger_interval_next_selected(plan, logical_chunk + 1u);
  s_source_chunk_index = s_emit_chunk;
}

static void IRAM_ATTR process_s31_dump_ring(const capture_config_t *config,
                                            const trigger_plan_t *plan) {
  if (s31_continuous_real_if_selected(config)) {
    /* USB transports the 4 MSa/s signed ADC stream losslessly and leaves the
     * Fs/4 analytic conversion to Soapy. This removes the firmware FIR from
     * the acquisition critical path while retaining 2 MSa/s complex output. */
    s31_process_parlio_diag(config);
    return;
  }
  if (s31_parlio_native_packed_iq_selected(config) ||
      config->rx_filter.rx_filter_override == S31_GPIO_DIAG_OVERRIDE ||
      config->rx_filter.rx_filter_override == S31_PARLIO_NATIVE_IQ_OVERRIDE ||
      config->rx_filter.rx_filter_override == S31_PARLIO_HOST_IQ_OVERRIDE ||
      config->rx_filter.rx_filter_override == S31_HP_TCM_PROBE_OVERRIDE) {
    s31_process_parlio_diag(config);
    return;
  }
#if CONFIG_ESP_SDR_TRANSPORT_USB
  if (s31_modem_diag_probe_enabled(config)) {
    s31_process_modem_diag_probe(config);
    return;
  }
#endif
  uint64_t abs_write = s31_current_abs_write(config);
  uint32_t guard_words = s31_ring_guard_words(plan);
  s_s31_diag_emit_chunk = s_emit_chunk;
  s_s31_diag_ring_started = s_s31_ring_started ? 1u : 0u;
  if (s31_pulse_tcm_before_copy(config) &&
      plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
    process_s31_live_pulse_interval(config, plan, abs_write);
    return;
  }
  if (abs_write <= guard_words) {
    s_s31_diag_c_hi = 0u;
    return;
  }
  uint32_t output_words = s31_output_chunk_words(config);
  uint64_t hi = abs_write - guard_words;
  uint32_t c_hi = (uint32_t)(hi / output_words);
  s_s31_diag_c_hi = c_hi;
  if (!s_s31_ring_started) {
    s_emit_chunk = c_hi > 0u ? c_hi - 1u : 0u;
    if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
      s_emit_chunk = trigger_interval_next_selected(plan, s_emit_chunk);
    }
    s_s31_ring_started = true;
    s_s31_diag_ring_started = 1u;
    s_s31_diag_emit_chunk = s_emit_chunk;
  }
  uint64_t emit_abs = (uint64_t)s_emit_chunk * output_words;
  if (hi > emit_abs && (hi - emit_abs) > (DUMP_BANK_WORDS - guard_words)) {
    uint64_t new_emit_abs = hi - (DUMP_BANK_WORDS / 2u);
    uint32_t new_emit_chunk = (uint32_t)(new_emit_abs / output_words);
    if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
      s_stream_dropped_chunks +=
          trigger_interval_selected_count(plan, s_emit_chunk, new_emit_chunk);
    } else {
      s_stream_dropped_chunks += new_emit_chunk - s_emit_chunk;
    }
    s_emit_chunk = new_emit_chunk;
    if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
      s_emit_chunk = trigger_interval_next_selected(plan, s_emit_chunk);
    }
    s_s31_diag_emit_chunk = s_emit_chunk;
  }
  if (c_hi <= s_emit_chunk) {
    return;
  }

  uint32_t sample_cycles = adc_dump_sample_cycles(config);
  uint32_t sample_rate_hz =
      sample_cycles != 0u ? ADC_DUMP_CLOCK_HZ / sample_cycles : 0u;
  uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
  uint32_t rx_gain = agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M;
  uint32_t agc_state = (agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                       MODEM_WIFI_AGC_AGCRD3_STATE_S;

  if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
    if (plan->interval_chunks == 1u && plan->interval_duration_chunks != 0u) {
      uint64_t low_abs = abs_write > DUMP_BANK_WORDS
                             ? abs_write - DUMP_BANK_WORDS + output_words
                             : 0u;
      uint32_t low_chunk =
          (uint32_t)((low_abs + output_words - 1u) / output_words);
      if (s_emit_chunk < low_chunk) {
        s_stream_dropped_chunks += low_chunk - s_emit_chunk;
        s_emit_chunk = low_chunk;
      }
      if (s_emit_chunk >= c_hi) {
        s_source_chunk_index = s_emit_chunk;
        return;
      }
      uint32_t n = c_hi - s_emit_chunk;
      if (n > S31_RING_MAX_CHUNKS_PER_POLL) {
        n = S31_RING_MAX_CHUNKS_PER_POLL;
      }
      uint32_t available = stream_ring_available_slots();
      if (n > available) {
        n = available;
      }
      if (n == 0u) {
        s_source_chunk_index = s_emit_chunk;
        return;
      }
      stream_frame_t *slots[S31_RING_MAX_CHUNKS_PER_POLL];
      if (!stream_ring_reserve_slots(n, slots)) {
        return;
      }
      for (uint32_t i = 0u; i < n; ++i) {
        uint32_t source_chunk = s_emit_chunk + i;
        const uint32_t *words = s31_exact_chunk_source(config, source_chunk);
        fill_and_push_chunk(config, words, source_chunk, sample_rate_hz,
                            rx_gain, agc_state, slots[i]);
        s31_chunk_copy_done(source_chunk);
      }
      stream_ring_commit_reserved(n);
      s_emit_chunk += n;
      s_source_chunk_index = s_emit_chunk;
      s_s31_diag_emit_chunk = s_emit_chunk;
      return;
    }
    uint32_t latest_selected;
    if (!trigger_interval_prev_selected(plan, c_hi, &latest_selected)) {
      return;
    }
    uint64_t low_abs = abs_write > DUMP_BANK_WORDS
                           ? abs_write - DUMP_BANK_WORDS + output_words
                           : 0u;
    uint32_t low_chunk =
        (uint32_t)((low_abs + output_words - 1u) / output_words);
    if (latest_selected < low_chunk) {
      s_stream_dropped_chunks +=
          trigger_interval_selected_count(plan, s_emit_chunk, low_chunk);
      s_emit_chunk = trigger_interval_next_selected(plan, low_chunk);
      s_source_chunk_index = s_emit_chunk;
      s_s31_diag_emit_chunk = s_emit_chunk;
      return;
    }
    if (latest_selected < s_emit_chunk) {
      s_source_chunk_index = s_emit_chunk;
      s_s31_diag_emit_chunk = s_emit_chunk;
      return;
    }
    uint32_t older_selected =
        trigger_interval_selected_count(plan, s_emit_chunk, latest_selected);
    if (older_selected != 0u) {
      s_stream_dropped_chunks += older_selected;
    }
    stream_frame_t *slots[1];
    if (!stream_ring_reserve_slots(1u, slots)) {
      ++s_stream_dropped_chunks;
      return;
    }
    const uint32_t *words = s31_exact_chunk_source(config, latest_selected);
    fill_and_push_chunk(config, words, latest_selected, sample_rate_hz, rx_gain,
                        agc_state, slots[0]);
    s31_chunk_copy_done(latest_selected);
    stream_ring_commit_reserved(1u);
    s_emit_chunk = trigger_interval_next_selected(plan, latest_selected + 1u);
    s_source_chunk_index = s_emit_chunk;
    s_s31_diag_emit_chunk = s_emit_chunk;
    return;
  }

  uint32_t n = c_hi - s_emit_chunk;
  if (n > S31_RING_MAX_CHUNKS_PER_POLL) {
    n = S31_RING_MAX_CHUNKS_PER_POLL;
  }

  int32_t dc_i = s_power_trigger_dc_i_q16 >> 16;
  int32_t dc_q = s_power_trigger_dc_q_q16 >> 16;
  bool dc_valid = s_power_trigger_dc_valid;
  int32_t sum_i = 0, sum_q = 0;
  uint32_t sum_n = 0u;
  for (uint32_t k = 0u; k < n; ++k) {
    uint32_t c = s_emit_chunk + k;
    if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
      s_chunk_mark[k] = trigger_interval_select(plan, c) ? CHUNK_STREAM_TRIGGER
                                                         : CHUNK_STREAM_SKIP;
    } else {
      const uint32_t *words =
          s31_chunk_source(config, c, abs_write, true);
      s_chunk_mark[k] = chunk_triggers(plan, words, &sum_i, &sum_q, &sum_n,
                                       dc_i, dc_q, dc_valid)
                            ? CHUNK_STREAM_TRIGGER
                            : CHUNK_STREAM_SKIP;
    }
  }
  if (plan->mode != IQ_TRIGGER_MODE_INTERVAL) {
    expand_pre_post(s_chunk_mark, n, plan->pre_chunks, plan->post_chunks);
    if (plan->mode == IQ_TRIGGER_MODE_POWER && sum_n != 0u) {
      int32_t mean_i = (sum_i / (int32_t)sum_n) * 65536;
      int32_t mean_q = (sum_q / (int32_t)sum_n) * 65536;
      if (!s_power_trigger_dc_valid) {
        s_power_trigger_dc_i_q16 = mean_i;
        s_power_trigger_dc_q_q16 = mean_q;
        s_power_trigger_dc_valid = true;
      } else {
        s_power_trigger_dc_i_q16 +=
            (mean_i - s_power_trigger_dc_i_q16) >> plan->dc_shift;
        s_power_trigger_dc_q_q16 +=
            (mean_q - s_power_trigger_dc_q_q16) >> plan->dc_shift;
      }
    }
  }

  uint32_t selected = 0u;
  for (uint32_t k = 0u; k < n; ++k) {
    if (s_chunk_mark[k] != CHUNK_STREAM_SKIP) {
      ++selected;
    }
  }
  uint32_t available = stream_ring_available_slots();
  uint32_t reserve = selected < available ? selected : available;
  if (reserve < selected) {
    s_stream_dropped_chunks += selected - reserve;
  }
  stream_frame_t *slots[IQ_STREAM_RING_CHUNKS];
  if (reserve != 0u && !stream_ring_reserve_slots(reserve, slots)) {
    s_stream_dropped_chunks += reserve;
    reserve = 0u;
  }
  uint32_t filled = 0u;
  for (uint32_t k = 0u; k < n && filled < reserve; ++k) {
    if (s_chunk_mark[k] == CHUNK_STREAM_SKIP) {
      continue;
    }
    uint32_t c = s_emit_chunk + k;
    const uint32_t *words = s31_chunk_source(config, c, abs_write, true);
    uint32_t source_chunk = s_s31_copy_source_chunk;
    fill_and_push_chunk(config, words, source_chunk, sample_rate_hz, rx_gain,
                        agc_state, slots[filled]);
    s31_chunk_copy_done(c);
    ++filled;
  }
  if (filled != 0u) {
    stream_ring_commit_reserved(filled);
  }
  if (reserve > filled) {
    stream_ring_release_reserved(reserve - filled);
  }
  s_emit_chunk += n;
  s_source_chunk_index = s_emit_chunk;
  s_s31_diag_emit_chunk = s_emit_chunk;
}

static void s31_wait_until_next_ring_window(const capture_config_t *config,
                                            const trigger_plan_t *plan) {
#if CONFIG_ESP_SDR_TRANSPORT_USB
  if (s31_modem_diag_probe_enabled(config)) {
    taskYIELD();
    return;
  }
#endif
  if (s31_pulse_tcm_before_copy(config) &&
      plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
    if (s31_pipelined_stage_enabled(config) &&
        s31_software_decimation(config) == 1u && !stream_output_int8()) {
      /* The staged transaction already waited for the writer to acquire its
       * complete next block. s_s31_diag_abs_write predates that transaction,
       * so using it below would sleep for the same block a second time. */
      taskYIELD();
      return;
    }
    uint64_t abs_write = s_s31_diag_abs_write;
    uint32_t next_chunk = s_emit_chunk;
    uint32_t output_words = s31_output_chunk_words(config);
    int32_t offset = s31_live_copy_offset_chunks(config);
    int64_t copy_chunk = (int64_t)next_chunk - offset;
    if (copy_chunk < 0) {
      copy_chunk = 0;
    }
    uint64_t ready_abs =
        ((uint64_t)copy_chunk + 1u) * output_words +
        S31_LIVE_COPY_GUARD_WORDS;
    if (ready_abs <= abs_write) {
      taskYIELD();
      return;
    }
    uint64_t remaining_words64 = ready_abs - abs_write;
    uint32_t remaining_words = remaining_words64 > UINT32_MAX
                                   ? UINT32_MAX
                                   : (uint32_t)remaining_words64;
    uint32_t sample_cycles = adc_dump_write_sample_cycles(config);
    uint32_t remaining_us =
        sample_cycles != 0u
            ? (remaining_words * sample_cycles) /
                  (ADC_DUMP_CLOCK_HZ / 1000000u)
            : 0u;
    if (remaining_us > S31_POLL_WAKE_MARGIN_US + 1000u) {
      vTaskDelay(
          pdMS_TO_TICKS((remaining_us - S31_POLL_WAKE_MARGIN_US) / 1000u));
    } else if (remaining_us > S31_POLL_CLOSE_US) {
      esp_rom_delay_us(remaining_us - S31_POLL_CLOSE_US);
    } else {
      esp_rom_delay_us(S31_POLL_CLOSE_US);
    }
    return;
  }
  uint64_t abs_write = s_s31_diag_abs_write;
  uint32_t next_chunk = s_emit_chunk;
  uint32_t output_words = s31_output_chunk_words(config);
  if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
    next_chunk = trigger_interval_next_selected(plan, next_chunk);
  }
  uint64_t ready_abs = ((uint64_t)next_chunk + 1u) * output_words +
                       s31_ring_guard_words(plan);
  if (ready_abs <= abs_write) {
    taskYIELD();
    return;
  }
  uint64_t remaining_words64 = ready_abs - abs_write;
  uint32_t remaining_words = remaining_words64 > UINT32_MAX
                                 ? UINT32_MAX
                                 : (uint32_t)remaining_words64;
  uint32_t sample_cycles = adc_dump_write_sample_cycles(config);
  uint32_t remaining_us =
      sample_cycles != 0u
          ? (remaining_words * sample_cycles) /
                (ADC_DUMP_CLOCK_HZ / 1000000u)
          : 0u;
  if (remaining_us > S31_POLL_WAKE_MARGIN_US + 1000u) {
    vTaskDelay(pdMS_TO_TICKS((remaining_us - S31_POLL_WAKE_MARGIN_US) / 1000u));
  } else if (remaining_us > S31_POLL_CLOSE_US) {
    esp_rom_delay_us(remaining_us - S31_POLL_CLOSE_US);
    taskYIELD();
  } else {
    esp_rom_delay_us(S31_POLL_CLOSE_US);
    taskYIELD();
  }
}
#endif

static void IRAM_ATTR producer_task(void *arg) {
  (void)arg;
  volatile uint32_t stack_probe = 0u;
  ESP_LOGI("iq_capture", "producer stack %p (%s)", &stack_probe,
           esp_ptr_external_ram((const void *)&stack_probe) ? "PSRAM" : "TCM");
  ESP_LOGI("iq_capture", "producer running on core %d", xPortGetCoreID());
#if CONFIG_IDF_TARGET_ESP32S31
  /* Trigger FreeRTOS's lazy PIE-context setup while scheduling and interrupts
   * are still available. The staged transaction can then use PIE safely with
   * both cores quiesced. */
  s31_pie_memcpy_aligned((void *)(uintptr_t)S31_STAGE_BASE,
                         (const void *)(uintptr_t)S31_STAGE_BASE, 64u);
#endif
  while (true) {
    if (!iq_network_stream_armed()) {
      vTaskDelay(pdMS_TO_TICKS(IQ_TASK_IDLE_DELAY_MS));
      continue;
    }
    if (config_apply_in_progress()) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
#if CONFIG_IDF_TARGET_ESP32S31
    capture_config_t config;
    trigger_plan_t plan;
    taskENTER_CRITICAL(&s_config_mux);
    config = s_config;
    plan = s_trigger_plan;
    taskEXIT_CRITICAL(&s_config_mux);
    if (!capture_engine_running(&config)) {
      engine_enable();
      stream_state_reset();
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    (void)ulTaskNotifyTake(pdTRUE, 0u);
    s_producer_active = true;
    process_s31_dump_ring(&config, &plan);
    s_producer_active = false;
    if (!s31_continuous_real_if_selected(&config) &&
        !s31_parlio_native_packed_iq_selected(&config)) {
      s31_wait_until_next_ring_window(&config, &plan);
    }
#endif
  }
}

/* ---- config apply ---- */

static capture_config_t active_config_snapshot(void) {
  capture_config_t config;
  taskENTER_CRITICAL(&s_config_mux);
  config = s_config;
  taskEXIT_CRITICAL(&s_config_mux);
  return config;
}

static capture_config_t sanitize_capture_config(capture_config_t config) {
#if CONFIG_IDF_TARGET_ESP32S31

  if (config.radio.rf_freq_hz < RF_FREQ_MIN_HZ)
    config.radio.rf_freq_hz = RF_FREQ_MIN_HZ;
  if (config.radio.rf_freq_hz > RF_FREQ_MAX_HZ)
    config.radio.rf_freq_hz = RF_FREQ_MAX_HZ;
  /* phy_set_freq() accepts a signed kHz remainder; make HTTP readback reflect
   * the actual hardware resolution rather than preserving unattainable Hz. */
  config.radio.rf_freq_hz =
      ((config.radio.rf_freq_hz + 500u) / 1000u) * 1000u;
  config.bandwidth.bw_mhz = 20u;
  config.bandwidth.second_chan = SECOND_CHAN_NONE;
#endif
  config.dc_offset.automatic = config.dc_offset.automatic != 0u;
  if (config.rx_filter.rx_filter_override == 0u &&
      config.rx_filter.filter_bw_mhz != RX_FILTER_BW_OPEN) {
    if (config.rx_filter.filter_bw_mhz < RX_FILTER_BW_MIN_MHZ)
      config.rx_filter.filter_bw_mhz = RX_FILTER_BW_MIN_MHZ;
    if (config.rx_filter.filter_bw_mhz > RX_FILTER_BW_MAX_MHZ)
      config.rx_filter.filter_bw_mhz = RX_FILTER_BW_MAX_MHZ;
  }
  if (config.rx_filter.rx_filter_dcap > 63u)
    config.rx_filter.rx_filter_dcap = 63u;
  if (config.rx_filter.rx_filter_override != S31_GPIO_DIAG_OVERRIDE &&
      config.rx_filter.rx_filter_override != S31_PARLIO_NATIVE_IQ_OVERRIDE &&
      config.rx_filter.rx_filter_override != S31_PARLIO_HOST_IQ_OVERRIDE &&
      config.rx_filter.rx_filter_override != S31_HP_TCM_PROBE_OVERRIDE &&
      config.rx_filter.rx_filter_mode > 32u)
    config.rx_filter.rx_filter_mode = 32u;
  return config;
}

static void queue_config_apply(const capture_config_t *config) {
  capture_config_t sanitized = sanitize_capture_config(*config);
  trigger_plan_t plan = build_trigger_plan(&sanitized);
  taskENTER_CRITICAL(&s_config_mux);
  s_pending_config = sanitized;
  s_pending_trigger_plan = plan;
  s_config_apply_pending = true;
  taskEXIT_CRITICAL(&s_config_mux);
}

static bool consume_pending_config_for_apply(void) {
  bool pending;
  int64_t now_us = esp_timer_get_time();
  taskENTER_CRITICAL(&s_config_mux);
  pending = s_config_apply_enabled && s_config_apply_pending &&
            !s_config_apply_in_progress;
  if (pending) {
    s_config = s_pending_config;
    s_trigger_plan = s_pending_trigger_plan;
    s_config_apply_pending = false;
    s_config_apply_in_progress = true;
    s_config_apply_started_us = now_us;
  }
  taskEXIT_CRITICAL(&s_config_mux);
  return pending;
}

static void finish_config_apply(void) {
  taskENTER_CRITICAL(&s_config_mux);
  s_config_apply_in_progress = false;
  s_config_apply_started_us = 0;
  taskEXIT_CRITICAL(&s_config_mux);
}

static bool config_apply_in_progress(void) {
  bool in_progress;
  taskENTER_CRITICAL(&s_config_mux);
  in_progress = s_config_apply_pending || s_config_apply_in_progress;
  taskEXIT_CRITICAL(&s_config_mux);
  return in_progress;
}

static void recover_stale_config_apply(void) {
  int64_t now_us = esp_timer_get_time();
  taskENTER_CRITICAL(&s_config_mux);
  if (s_config_apply_in_progress && s_config_apply_started_us != 0 &&
      now_us - s_config_apply_started_us > 500000) {
    s_config_apply_in_progress = false;
    s_config_apply_started_us = 0;
  }
  taskEXIT_CRITICAL(&s_config_mux);
}

enum {
  CONFIG_STAGE_PACKET_ACCEPTED = 0u,
  CONFIG_STAGE_AFTER_STREAM_ARM = 8u,
  CONFIG_STAGE_AFTER_QUEUE = 9u,
  CONFIG_STAGE_APPLY_START = 1u,
  CONFIG_STAGE_AFTER_WIFI = 2u,
  CONFIG_STAGE_BEFORE_MODEM_APPLY = 3u,
  CONFIG_STAGE_AFTER_MODEM_APPLY = 4u,
  CONFIG_STAGE_BEFORE_RESTART = 6u,
  CONFIG_STAGE_AFTER_RESTART = 7u,
  CONFIG_STAGE_RESTART_ENTRY = 20u,
  CONFIG_STAGE_AFTER_ENGINE_ENABLE = 21u,
  CONFIG_STAGE_BEFORE_GAIN_TABLE = 22u,
  CONFIG_STAGE_AFTER_GAIN_TABLE = 23u,
  CONFIG_STAGE_AFTER_LIVE_GAIN = 24u,
  CONFIG_STAGE_AFTER_STREAM_STATE = 25u,
};

static void push_config_stage_report(const capture_config_t *config,
                                     uint32_t stage) {
  config_report_t report = {
      .magic = {'C', 'F', 'G', '1'},
      .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000),
      .stage = stage,
      .rx_gain = config->gain.rx_gain,
      .adc_source_sel = config->iq_engine.adc_source_sel,
      .expert_gain_word0 = config->gain.expert_gain_word0,
      .expert_gain_word1 = config->gain.expert_gain_word1,
      .ctrl = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL),
      .mode = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_MODE),
      .trigger_interval_chunks = config->trigger.trigger_config[0],
      .trigger_duration_chunks = config->trigger.trigger_config[2],
#if CONFIG_IDF_TARGET_ESP32S31
      .hp_tcm_dump_ctrl = reg32_read_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG),
#else
      .hp_tcm_dump_ctrl = 0u,
#endif
  };
  (void)stream_ring_push_config_report(&report);
}

static bool capture_engine_running(const capture_config_t *config) {
#if CONFIG_IDF_TARGET_ESP32S31

  uint32_t ctrl = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL);
  if ((ctrl & MODEM_WIFI_DUMP_CTRL_ENABLE_BIT) == 0u) {
    return false;
  }
  if ((ctrl & MODEM_WIFI_DUMP_CTRL_CONTINUOUS_TRIGGER_GATE_BIT) == 0u) {
    return false;
  }
  return true;
#else
  return true;
#endif
}

static bool config_equal_except_gain(const capture_config_t *a,
                                     const capture_config_t *b) {
  return memcmp(&a->stream, &b->stream, sizeof(a->stream)) == 0 &&
         memcmp(&a->radio, &b->radio, sizeof(a->radio)) == 0 &&
         memcmp(&a->bandwidth, &b->bandwidth, sizeof(a->bandwidth)) == 0 &&
         memcmp(&a->iq_engine, &b->iq_engine, sizeof(a->iq_engine)) == 0 &&
         memcmp(&a->trigger, &b->trigger, sizeof(a->trigger)) == 0 &&
         memcmp(&a->rx_filter, &b->rx_filter, sizeof(a->rx_filter)) == 0 &&
         memcmp(&a->dc_offset, &b->dc_offset, sizeof(a->dc_offset)) == 0;
}

static bool config_equal_except_trigger(const capture_config_t *a,
                                        const capture_config_t *b) {
  return memcmp(&a->stream, &b->stream, sizeof(a->stream)) == 0 &&
         memcmp(&a->radio, &b->radio, sizeof(a->radio)) == 0 &&
         memcmp(&a->gain, &b->gain, sizeof(a->gain)) == 0 &&
         memcmp(&a->bandwidth, &b->bandwidth, sizeof(a->bandwidth)) == 0 &&
         memcmp(&a->iq_engine, &b->iq_engine, sizeof(a->iq_engine)) == 0 &&
         memcmp(&a->rx_filter, &b->rx_filter, sizeof(a->rx_filter)) == 0 &&
         memcmp(&a->dc_offset, &b->dc_offset, sizeof(a->dc_offset)) == 0;
}

#if CONFIG_IDF_TARGET_ESP32S31
static bool config_equal_except_diag_mode(const capture_config_t *a,
                                          const capture_config_t *b) {
  capture_config_t left = *a;
  capture_config_t right = *b;
  left.rx_filter.rx_filter_mode = 0u;
  right.rx_filter.rx_filter_mode = 0u;
  return memcmp(&left, &right, sizeof(left)) == 0;
}
#endif

static void apply_live_gain_only_config(const capture_config_t *config) {
  push_config_stage_report(config, CONFIG_STAGE_APPLY_START);
  wait_stream_pipeline_idle();
  /* The calibrated gain table was populated after engine_enable(). Rewriting
   * gain RAM while the dump engine is active corrupts the RX operating point;
   * a live gain change only needs to select another existing table slot. */
  modem_config_t gain_cfg = modem_config_from_capture(config);
  modem_apply_live_rx_config(&gain_cfg);
  /* The forced-gain operating point changed: re-arm the DCOC servo on the
   * new slot (disarms in AGC modes). */
  dcoc_arm(&gain_cfg);
  push_config_stage_report(config, CONFIG_STAGE_AFTER_LIVE_GAIN);
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  wait_stream_pipeline_idle();
  stream_state_reset();
  push_config_stage_report(config, CONFIG_STAGE_AFTER_STREAM_STATE);
#endif
  push_config_stage_report(config, CONFIG_STAGE_AFTER_RESTART);
}

static void restart_capture(const capture_config_t *config) {
  push_config_stage_report(config, CONFIG_STAGE_RESTART_ENTRY);
  s_isr_prev_sa = 0u;
  s_isr_wrap_count = 0u;
  engine_enable();
  push_config_stage_report(config, CONFIG_STAGE_AFTER_ENGINE_ENABLE);
  /* The engine/FE bring-up leaves gain memory stale, so forced slot indices
   * can map to a flat (~2-level) gain. Regenerate the gain ramp
   * before selecting a manual gain.
   */
  if (config->gain.gain_mode == GAIN_MODE_MANUAL) {
    push_config_stage_report(config, CONFIG_STAGE_BEFORE_GAIN_TABLE);
    modem_setup_rx_gain_table();
    push_config_stage_report(config, CONFIG_STAGE_AFTER_GAIN_TABLE);
    modem_config_t gain_cfg = modem_config_from_capture(config);
    modem_apply_live_rx_config(&gain_cfg);
    push_config_stage_report(config, CONFIG_STAGE_AFTER_LIVE_GAIN);
  }
  /* Forced-gain modes: (re-)arm the DCOC servo for this operating point (the
   * freshly rebuilt table only carries the coarse boot-time compensation; at
   * high forced gains the leftover LO self-mixing DC clips the ADC). The
   * producer feeds it samples per emitted chunk; it converges within seconds
   * and keeps tracking drift. Arm handles AGC by disarming. */
  modem_config_t dcoc_cfg = modem_config_from_capture(config);
  dcoc_arm(&dcoc_cfg);
  wait_stream_pipeline_idle();
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
#endif
  stream_state_reset();
  push_config_stage_report(config, CONFIG_STAGE_AFTER_STREAM_STATE);
}

static void apply_pending_config(void) {
  static capture_config_t applied;
  static bool have_applied;
  capture_config_t cfg = active_config_snapshot();

  if (have_applied &&
      memcmp(&applied, &cfg, sizeof(cfg)) == 0 &&
      capture_engine_running(&cfg)) {
    return;
  }
#if CONFIG_IDF_TARGET_ESP32S31

  if (have_applied &&
      (applied.rx_filter.rx_filter_override == S31_GPIO_DIAG_OVERRIDE ||
       applied.rx_filter.rx_filter_override ==
           S31_PARLIO_NATIVE_IQ_OVERRIDE) &&
      applied.rx_filter.rx_filter_override ==
          cfg.rx_filter.rx_filter_override &&
      config_equal_except_diag_mode(&applied, &cfg) &&
      capture_engine_running(&cfg)) {
    wait_stream_pipeline_idle();
    s31_gpio_diag_stop();
    s31_configure_modem_diag(&cfg);
    s31_gpio_diag_prepare(&cfg);
    stream_state_reset();
    applied = cfg;
    return;
  }
  if (have_applied &&
      config_equal_except_trigger(&applied, &cfg) &&
      capture_engine_running(&cfg)) {
    stream_state_reset();
    applied = cfg;
    return;
  }
  if (have_applied &&
      config_equal_except_gain(&applied, &cfg) &&
      applied.gain.gain_mode == cfg.gain.gain_mode &&
      capture_engine_running(&cfg)) {
    apply_live_gain_only_config(&cfg);
    applied = cfg;
    have_applied = true;
    return;
  }
#endif
  push_config_stage_report(&cfg, CONFIG_STAGE_APPLY_START);
  wait_stream_pipeline_idle();
  /* Fractional/out-of-band RF programming touches the same closed modem path
   * as continuous IQ capture. Quiesce the dump engine before retuning; the
   * restart below re-arms it with the new configuration. */
  engine_disable();
  wifi_rx_config_t wifi_config = wifi_rx_config_from_capture(&cfg);
  wifi_rx_apply_config(&wifi_config);
  if (wifi_config.stream_packets != 0u) {
    wifi_rx_init(stream_ring_push_wifi);
  }
  push_config_stage_report(&cfg, CONFIG_STAGE_AFTER_WIFI);

  modem_prepare_direct_phy_access();
  modem_config_t modem_config = modem_config_from_capture(&cfg);
  push_config_stage_report(&cfg, CONFIG_STAGE_BEFORE_MODEM_APPLY);
  {
    modem_apply_rx_config(&modem_config);
  }
  push_config_stage_report(&cfg, CONFIG_STAGE_AFTER_MODEM_APPLY);
  push_config_stage_report(&cfg, CONFIG_STAGE_BEFORE_RESTART);
  restart_capture(&cfg);
  push_config_stage_report(&cfg, CONFIG_STAGE_AFTER_RESTART);

  applied = cfg;
  have_applied = true;
}

static void service_pending_config(void) {
  recover_stale_config_apply();
  if (consume_pending_config_for_apply()) {
    apply_pending_config();
    finish_config_apply();
  }
}

/* ---- HTTP configuration callbacks and UDP stream ---- */

static void network_get_config(capture_config_t *config) {
  *config = active_config_snapshot();
}

static void network_apply_config(const capture_config_t *config) {
  capture_config_t sanitized = sanitize_capture_config(*config);
  wifi_rx_set_stream_armed(true);
  queue_config_apply(&sanitized);
}

static uint32_t network_dropped_chunks(void) { return s_stream_dropped_chunks; }

static uint32_t network_source_chunk(void) { return s_source_chunk_index; }

static uint32_t network_adc_dump_cfg(void) {
  return reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CFG);
}

static uint32_t network_adc_dump_mode(void) {
  return reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_MODE);
}

static uint32_t network_modem_diag_fix_sel(void) {
  return reg32_read_addr(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG);
}

static uint32_t network_modem_diag_exchange(void) {
  return reg32_read_addr(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG);
}

static uint32_t network_bb_diag(void) {
  return reg32p_read(&MODEM_WIFI_BB.BB_DIAG0);
}

static uint32_t network_dcoc_diag(void) { return dcoc_diag(); }
static bool network_dcoc_active(void) { return dcoc_active(); }

static void network_set_capture_armed(bool armed) {
  s_network_capture_armed = armed;
  if (armed) {
    capture_config_t config = active_config_snapshot();
    /* Capture is stopped when UDP is stopped.  Serialize a fresh engine start
     * in the stream task; UDP arming is delayed long enough for this restart
     * and its configuration reports to finish before the producer runs. */
    queue_config_apply(&config);
  } else {
#if CONFIG_IDF_TARGET_ESP32S31
    /* The persistent PARLIO transaction otherwise keeps overwriting its DMA
     * ring while no host owns the stream. A later START then observes roughly
     * the entire 100 ms arm delay as a synthetic source gap. Stop at ownership
     * release; s31_process_parlio_diag() restarts the retained unit on demand. */
    wait_stream_pipeline_idle();
    s31_gpio_diag_stop();
#endif
  }
}

static bool network_config_applying(void) {
  bool applying;
  taskENTER_CRITICAL(&s_config_mux);
  applying = s_config_apply_pending || s_config_apply_in_progress;
  taskEXIT_CRITICAL(&s_config_mux);
  return applying;
}

#include "s31_burst.h"

static void stream_next_frame(void) {
  if (config_apply_in_progress()) {
    vTaskDelay(pdMS_TO_TICKS(1));
    return;
  }
  stream_frame_t *frame = stream_ring_peek();
  if (frame == NULL) {
    /* Wake immediately when the producer commits instead of a fixed sleep:
     * the USB transport's drain rate barely exceeds the full-rate arrival
     * rate, so any fixed sleep here turns into source drops. */
    (void)stream_ring_wait_frames(1u);
    frame = stream_ring_peek();
    if (frame == NULL) {
      return;
    }
  }
  s_stream_write_active = true;
  if (config_apply_in_progress()) {
    s_stream_write_active = false;
    vTaskDelay(pdMS_TO_TICKS(1));
    return;
  }
  size_t frame_size = stream_frame_wire_size(frame);
  if (frame_size == 0u) {
    stream_ring_pop();
    s_stream_write_active = false;
    return;
  }
  bool sent = false;
  if(iq_network_stream_owner()==IQ_STREAM_OWNER_SERIAL)
    sent=s31_burst_frame(frame);
#if CONFIG_ESP_SDR_TRANSPORT_USB
  if (iq_network_stream_owner() == IQ_STREAM_OWNER_USB) {
    sent = iq_usb_send_frame((const uint8_t *)frame, frame_size);
  }
#endif
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  if (iq_network_stream_owner() == IQ_STREAM_OWNER_ETH) {
  if (iq_network_stream_format() == IQ_USB_FORMAT_INT8 &&
      memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_IQ, 4u) == 0) {
    sent = iq_network_send_frame_int8((const uint8_t *)frame, frame_size);
  } else {
    /* Fill one hardware kick up to the default 12-descriptor ring. IQC8 uses
     * two descriptors and the larger, packet-efficient IQR8 uses three.
     * Non-compact or mixed report traffic remains a single-frame operation. */
    enum { MAX_ETH_BATCH_FRAMES = CONFIG_ETH_DMA_TX_BUFFER_NUM / 2u };
    stream_frame_t *batch_frames[MAX_ETH_BATCH_FRAMES];
    const uint8_t *batch_data[MAX_ETH_BATCH_FRAMES];
    size_t batch_sizes[MAX_ETH_BATCH_FRAMES];
    uint32_t batch_count = 1u;
    if ((memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_IQ8, 4u) == 0 ||
         memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_REAL8, 4u) == 0) &&
        MAX_ETH_BATCH_FRAMES > 1u) {
      uint32_t available =
          stream_ring_peek_batch(batch_frames, MAX_ETH_BATCH_FRAMES);
      batch_count = 0u;
      uint32_t descriptors = 0u;
      const uint32_t descriptor_limit =
          CONFIG_ETH_DMA_TX_BUFFER_NUM > 3u
              ? CONFIG_ETH_DMA_TX_BUFFER_NUM - 3u
              : CONFIG_ETH_DMA_TX_BUFFER_NUM;
      for (uint32_t i = 0u; i < available; ++i) {
        size_t size = stream_frame_wire_size(batch_frames[i]);
        uint32_t fragments =
            (size + IQ_UDP_FRAGMENT_PAYLOAD_BYTES - 1u) /
            IQ_UDP_FRAGMENT_PAYLOAD_BYTES;
        if ((size != IQ8_FRAME_WIRE_BYTES &&
             size != REAL8_FRAME_WIRE_BYTES) ||
            (memcmp(batch_frames[i]->iq.magic, STREAM_FRAME_MAGIC_IQ8, 4u) !=
                 0 &&
             memcmp(batch_frames[i]->iq.magic, STREAM_FRAME_MAGIC_REAL8, 4u) !=
                 0) ||
            descriptors + fragments > descriptor_limit) {
          break;
        }
        batch_data[batch_count] = (const uint8_t *)batch_frames[i];
        batch_sizes[batch_count] = size;
        descriptors += fragments;
        ++batch_count;
      }
    }
    if (batch_count == 1u) {
      sent = iq_network_send_frame((const uint8_t *)frame, frame_size);
    } else {
      sent = iq_network_send_frames(batch_data, batch_sizes, batch_count);
    }
    if (sent || !iq_network_stream_armed()) {
      stream_ring_pop_batch(batch_count);
    } else {
      /* A transient descriptor/socket stall must not turn into a silent RF
       * hole. Leave every frame owned by the ring and retry. A partially sent
       * UDP batch can create duplicates, which the host de-duplicates; it
       * cannot create a missing source chunk. */
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    s_stream_write_active = false;
    return;
  }
  }
#endif
  if (sent || !iq_network_stream_armed()) {
    stream_ring_pop();
  } else {
    /* Preserve the slot across transient USB/Ethernet backpressure. */
    s_stream_write_active = false;
    vTaskDelay(pdMS_TO_TICKS(1));
    return;
  }
#if CONFIG_ESP_SDR_TRANSPORT_USB
  if (iq_network_stream_owner() == IQ_STREAM_OWNER_USB &&
      stream_ring_peek() == NULL) {
    iq_usb_flush();
  }
#endif
  s_stream_write_active = false;
}

void app_main(void) {
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
#endif
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("iq_network", ESP_LOG_INFO);
  (void)esp_task_wdt_deinit();
  modem_prepare_direct_phy_access();
  stream_ring_init_storage();
#if CONFIG_IDF_TARGET_ESP32S31 && CONFIG_ESP_SDR_TRANSPORT_USB && \
    CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  s_s31_parlio_dma_storage = heap_caps_aligned_calloc(
      64u, 1u, S31_PARLIO_DMA_BYTES,
      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  ESP_ERROR_CHECK(s_s31_parlio_dma_storage != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  ESP_LOGI("iq_capture", "reserved persistent PARLIO DMA ring at %p",
           s_s31_parlio_dma_storage);
#endif
  init_idf_services();

  const iq_network_callbacks_t network_callbacks = {
      .get_config = network_get_config,
      .apply_config = network_apply_config,
      .get_firmware_dropped_chunks = network_dropped_chunks,
      .get_source_chunk_index = network_source_chunk,
      .get_adc_dump_cfg = network_adc_dump_cfg,
      .get_adc_dump_mode = network_adc_dump_mode,
      .get_modem_diag_fix_sel = network_modem_diag_fix_sel,
      .get_modem_diag_exchange = network_modem_diag_exchange,
      .get_bb_diag = network_bb_diag,
      .get_dcoc_diag = network_dcoc_diag,
      .get_dcoc_active = network_dcoc_active,
      .is_config_applying = network_config_applying,
      .set_capture_armed = network_set_capture_armed,
  };
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  /* GMAC needs a contiguous pool of DMA-capable internal RAM. Install it
   * before Wi-Fi/PHY capture support consumes and fragments that heap. */
  iq_network_init(&network_callbacks);
#else
  /* The USB control protocol reuses the JSON builders and stream-owner state
   * from iq_network, but must not bring up the Ethernet MAC. */
  iq_control_init(&network_callbacks);
#endif

#if CONFIG_IDF_TARGET_ESP32S31
  /* The combined Ethernet/USB image leaves only a small DMA-capable internal
   * heap after Wi-Fi and TinyUSB start.  PARLIO keeps its acquisition ring in
   * PSRAM, but the IDF driver requires a contiguous internal descriptor list.
   * Hold a small block across stack initialization, then hand that exact hole
   * to the first host-selected PARLIO geometry in prepare(). */
  s_s31_parlio_internal_dma_reserve = heap_caps_aligned_alloc(
      64u, 2048u,
      MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  ESP_ERROR_CHECK(s_s31_parlio_internal_dma_reserve != NULL
                      ? ESP_OK
                      : ESP_ERR_NO_MEM);
#endif

#if CONFIG_IDF_TARGET_ESP32S31 && CONFIG_ESP_PHY_ENABLE_CERT_TEST
  init_s31_rftest_services();
#endif

  wifi_rx_config_t wifi_config = wifi_rx_config_from_capture(&s_config);
  ESP_LOGI("iq_capture", "initializing Wi-Fi capture support");
  wifi_rx_init(stream_ring_push_wifi);
  ESP_LOGI("iq_capture", "applying Wi-Fi capture config");
  wifi_rx_apply_config(&wifi_config);

  modem_config_t modem_config = modem_config_from_capture(&s_config);
  ESP_LOGI("iq_capture", "initializing direct modem access");
  ESP_ERROR_CHECK(modem_init(&modem_config));
  ESP_LOGI("iq_capture", "applying direct modem config");
  modem_apply_rx_config(&modem_config);
  ESP_LOGI("iq_capture", "direct modem config ready");
#if !CONFIG_IDF_TARGET_ESP32S31
  modem_calibrate_dco();
#endif
#if CONFIG_IDF_TARGET_ESP32S31
  s31_tcm_gate_watchdog_init();
#endif
  s_trigger_plan = build_trigger_plan(&s_config);
  s_config_apply_enabled = true;

#if CONFIG_ESP_SDR_TRANSPORT_USB
  iq_usb_init(&network_callbacks);
#endif
  ESP_ERROR_CHECK(
      xTaskCreatePinnedToCoreWithCaps(
          producer_task, "iq_producer", IQ_CHUNK_PRODUCER_TASK_STACK_BYTES,
          NULL, IQ_CHUNK_PRODUCER_TASK_PRIORITY, &s_producer_task_handle, 1,
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == pdPASS
          ? ESP_OK
          : ESP_ERR_NO_MEM);

#if !CONFIG_IDF_TARGET_ESP32S31
  restart_capture(&s_config);
#endif

  vTaskPrioritySet(NULL, STREAM_TASK_PRIORITY);
  ESP_LOGI("iq_capture", "stream running on core %d", xPortGetCoreID());
  s31_burst_init();
  while (true) {
    s31_burst_poll();
    service_pending_config();
    if (!s_network_capture_armed && capture_engine_running(&s_config)
    ) {
      reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                        MODEM_WIFI_DUMP_CTRL_ENABLE_BIT);
      reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                        MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
#if CONFIG_IDF_TARGET_ESP32S31
      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
#endif
      stream_state_reset();
    }
    stream_next_frame();
  }
}
