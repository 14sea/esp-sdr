/* Serial snapshot adapter for the existing S31 PARLIO receiver. All entry
 * points run on the stream task; the producer retains exclusive acquisition
 * ownership. Accumulate contiguous IQC8 chunks in 32 KiB of PSRAM, then stop
 * capture before the much slower UART transfer. No second capture engine. */
#include "burst_serial.h"
#include "gaintable.h"
#define S31_BURST_SAMPLES 16384u
static uint8_t *s31_burst_data;
static unsigned s31_burst_wanted,s31_burst_used,s31_burst_next_chunk;
static int64_t s31_burst_started,s31_burst_lease;
static int s31_burst_owner=-1;
static bool s31_burst_complete;
static void s31_burst_reply(const char *s) { (void)burst_serial_send(s,strlen(s)); }
static unsigned s31_burst_gain_max(void) {
    unsigned n=gaintable_entry_count();return n?n-1:0;
}
static void s31_burst_init(void) {
    s31_burst_data=heap_caps_malloc(S31_BURST_SAMPLES*2,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    ESP_ERROR_CHECK(s31_burst_data?ESP_OK:ESP_ERR_NO_MEM);
    esp_log_level_set("*",ESP_LOG_NONE);
    burst_serial_init();
}
static bool s31_burst_frame(const stream_frame_t *frame) {
    if(!s31_burst_wanted || s31_burst_complete || !iq_network_stream_armed())return true;
    bool packed=!memcmp(frame->iq.magic,STREAM_FRAME_MAGIC_IQ8,4);
    if(!packed && memcmp(frame->iq.magic,STREAM_FRAME_MAGIC_IQ,4))return true;
    /* Never join samples across a producer overrun. */
    if(s31_burst_used && frame->iq.source_chunk_index!=s31_burst_next_chunk)s31_burst_used=0;
    s31_burst_next_chunk=frame->iq.source_chunk_index+1;
    unsigned n=s31_burst_wanted-s31_burst_used;
    if(n>IQ_CHUNK_SAMPLE_WORDS)n=IQ_CHUNK_SAMPLE_WORDS;
    uint8_t *dst=s31_burst_data+s31_burst_used*2;
    if(packed)memcpy(dst,frame->iq.samples,n*2);
    else for(unsigned j=0;j<n;j++) {
        /* Match the existing S31 IQC1 -> IQC8 wire conversion. */
        uint32_t w=frame->iq.samples[j];dst[2*j]=(w>>12)&255;dst[2*j+1]=(w>>2)&255;
    }
    s31_burst_used+=n;
    s31_burst_complete=s31_burst_used==s31_burst_wanted;
    return true;
}
static void s31_burst_poll(void) {
    int64_t now=esp_timer_get_time();
    if(s31_burst_wanted) {
        bool owner=iq_network_stream_owner()==IQ_STREAM_OWNER_SERIAL;
        if(s31_burst_complete || !owner || now-s31_burst_started>3000000) {
            if(owner)iq_network_stream_end();
            if(owner && s31_burst_complete) {
                char h[96];unsigned bytes=s31_burst_wanted*2;
                snprintf(h,sizeof(h),"DATA %u %08" PRIx32 " %" PRIi64 "\n",s31_burst_wanted,
                    esp_rom_crc32_le(0,s31_burst_data,bytes),now-s31_burst_started);
                s31_burst_reply(h);(void)burst_serial_send(s31_burst_data,bytes);
            } else s31_burst_reply(owner?"ERR capture_timeout\n":"ERR stream_replaced\n");
            s31_burst_wanted=0;s31_burst_complete=false;
            s31_burst_lease=esp_timer_get_time()+5000000;
        }
        return; /* Keep the reply port fixed until the complete DATA payload. */
    }
    if(now>=s31_burst_lease)s31_burst_owner=-1;
    char line[128],answer[224],extra;unsigned n,rate;uint64_t nonce;
    int status=burst_serial_poll_line(line,sizeof(line));
    if(!status)return;
    int port=burst_serial_port();
    if((s31_burst_owner>=0 && s31_burst_owner!=port) ||
       iq_network_stream_owner()!=IQ_STREAM_OWNER_NONE) {s31_burst_reply("ERR busy\n");return;}
    if(status<0){s31_burst_reply("ERR command_length\n");return;}
    s31_burst_owner=port;s31_burst_lease=now+5000000;
    capture_config_t c=active_config_snapshot();
    if(sscanf(line,"SYNC %" SCNu64 " %c",&nonce,&extra)==1) {
        snprintf(answer,sizeof(answer),"SYNC %" PRIu64 "\n",nonce);s31_burst_reply(answer);return;
    }
    if(!strcmp(line,"INFO")){s31_burst_reply("S31SDR 6 burst 16384\n");return;}
    if(!strcmp(line,"CAPS")){s31_burst_reply("CAPS RXLIMITS SERIALLEASE "
#if CONFIG_ESP_SDR_UART_ENABLED
        "DUALSERIAL "
#endif
        "GAIN HWAGC IQ8 TUNEEXT\n");return;}
    if(!strcmp(line,"RANGE?")){s31_burst_reply("RANGE 2300 2800 1\n");return;}
    if(!strcmp(line,"LIMITS?")) {
        snprintf(answer,sizeof(answer),"LIMITS {\"gain\":[0,%u,1],\"bandwidth\":[13,54,1,0],\"rates\":[16000000,8000000,4000000],\"bits\":[8]}\n",s31_burst_gain_max());
        s31_burst_reply(answer);return;
    }
    if(!strcmp(line,"GAIN?")) {
        bool manual=c.gain.gain_mode==GAIN_MODE_MANUAL;
        snprintf(answer,sizeof(answer),"GAIN %s %d 0 %u %u\n",manual?"MANUAL":"HARDWARE",manual?(int)c.gain.rx_gain:-1,s31_burst_gain_max(),manual?1:0);
        s31_burst_reply(answer);return;
    }
    if(!strcmp(line,"TRANSPORT?")) {
        snprintf(answer,sizeof(answer),"TRANSPORT %s %u\n",port==BURST_SERIAL_UART?"UART":"USB",burst_serial_baud());s31_burst_reply(answer);return;
    }
    if(!strcmp(line,"RELEASE")){s31_burst_owner=-1;s31_burst_reply("OK\n");return;}
    if(!strcmp(line,"GAIN HARDWARE"))c.gain.gain_mode=GAIN_MODE_HARDWARE;
    else if(sscanf(line,"GAIN MANUAL %u %c",&n,&extra)==1 && n<=s31_burst_gain_max()) {
        c.gain.gain_mode=GAIN_MODE_MANUAL;c.gain.rx_gain=n;
    } else if(sscanf(line,"FREQ %u %c",&n,&extra)==1 && n>=2300 && n<=2800)c.radio.rf_freq_hz=n*1000000u;
    else if(sscanf(line,"BANDWIDTH %u %c",&n,&extra)==1 && (!n || (n>=13 && n<=54))) {
        c.rx_filter.rx_filter_override=0;c.rx_filter.filter_bw_mhz=n;
    } else if(sscanf(line,"CAP16 %u %u %c",&n,&rate,&extra)==2 && n>=256 && n<=S31_BURST_SAMPLES && (rate==6 || rate==4 || rate==5)) {
        unsigned divider=rate==6?1:rate==4?2:4;
        /* Apply geometry before arming. The next loop services the queued
         * config, while stream_next_frame refuses frames during application. */
        c.iq_engine.adc_decimation=divider;c.rx_filter.rx_filter_override=0;
        if(c.iq_engine.adc_decimation!=s_config.iq_engine.adc_decimation || s_config.rx_filter.rx_filter_override!=0)
            queue_config_apply(&c);
        s31_burst_used=0;s31_burst_wanted=n;s31_burst_complete=false;s31_burst_started=now;
        iq_network_stream_set_format(IQ_USB_FORMAT_INT8);
        iq_network_stream_begin(IQ_STREAM_OWNER_SERIAL);iq_network_stream_arm();return;
    } else {s31_burst_reply("ERR command\n");return;}
    queue_config_apply(&c);s31_burst_reply("OK\n");
}
