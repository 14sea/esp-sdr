/* Shared snapshot RX gain control. Codes are PHY gain indices, not dB.
 * Hardware AGC controls gain unless a manual index is selected.
 * Keep the conservative tested range 0..50 (C5) / 0..55 (S3); do not index unverified table slots. */
extern void force_rx_gain(unsigned,unsigned,unsigned);
#if CONFIG_IDF_TARGET_ESP32C5
#define BURST_GAIN_MAX 50u
#define BURST_GAIN_REG 0x600a702cu
#else
#define BURST_GAIN_MAX 55u
#define BURST_GAIN_REG 0x6001c02cu
#endif
typedef enum { GAIN_MANUAL, GAIN_HARDWARE } burst_gain_mode_t;
static burst_gain_mode_t gain_mode=GAIN_HARDWARE;
static unsigned gain_code=40;
static void gain_apply(void) { force_rx_gain(gain_mode!=GAIN_HARDWARE,gain_code,0); }
static bool gain_command(const char *line) {
    unsigned code;char extra;
    if(!strcmp(line,"GAIN?")) {
        char h[64];snprintf(h,sizeof(h),"GAIN %s %d 0 %u %u\n",gain_mode==GAIN_HARDWARE?"HARDWARE":"MANUAL",gain_mode==GAIN_HARDWARE?-1:(int)gain_code,BURST_GAIN_MAX,(unsigned)((REG_READ(BURST_GAIN_REG)>>23)&1));reply(h);return true;
    }
    if(!strcmp(line,"GAIN HARDWARE")) {gain_mode=GAIN_HARDWARE;}
    else if(sscanf(line,"GAIN MANUAL %u %c",&code,&extra)==1 && code<=BURST_GAIN_MAX) {
        gain_mode=GAIN_MANUAL;gain_code=code;
    } else return false;
    gain_apply();reply("OK\n");return true;
}
