"""Exercise original ESP32 protocol parsing with the RF hardware calls stubbed."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

class ESP32Commands(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_protocol(self):
        source=(Path(__file__).resolve().parents[1]/'main/targets/esp32/receiver.c').read_text()
        prepare=source[source.index('static bool rx_ready;'):source.index('/* Clock selectors')]
        handler=prepare+source[source.index('extern void set_chanfreq('):source.index('void app_main(')]
        stub=r'''
#include <assert.h>
#include <stdbool.h>
bool rx_recalibration_stale(void) { return false; }
void rx_recalibrate(unsigned mhz) {}
#include <stdbool.h>
static bool burst_version_command(const char *s) { return false; }
static bool burst_gpio_command(const char *s) { return false; }
#include <inttypes.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#define CONFIG_IDF_TARGET_ESP32 1
#include "rx_bandwidth.h"
#include "rx_tuning.h"
#include "rx_lo.h"
unsigned ram_chip_i2c_readReg(unsigned b,unsigned h,unsigned r){return b==0x65?0x63:0xb0;}
void ram_chip_i2c_writeReg(unsigned b,unsigned h,unsigned r,unsigned v){}
static int pll_offset;
#define MAX_SAMPLES 16380u
#define RX_GAIN 0
#define REG_READ(r) (hardware_agc?0:1u<<23)
#define ESP_OK 0
#define WIFI_SECOND_CHAN_NONE 0
typedef int esp_err_t;
static unsigned gain_max=72, gain_code=40, captures, bits, clock_code, channel, pll, crystal, xtal=40, tunes;
static bool hardware_agc=true;
static int rx_filter=-1;
static unsigned rom_chip_i2c_readReg(unsigned block,unsigned host,unsigned reg){assert(block==0x67 && host==1 && (reg==1 || reg==2));return 24;}
static char response[256];
#define spectrum_acquire NULL
static unsigned frequency_mhz;
static bool spectrum_command(const char *s,unsigned f,void *acquire){return false;}
static void reply(const char *fmt,...) {va_list a;va_start(a,fmt);vsnprintf(response,sizeof(response),fmt,a);va_end(a);}
static unsigned burst_serial_baud(void){return 2000000;}
static void apply_gain(void){}
static void tune_rx(unsigned mhz);
static void rom_pbus_workmode(void){}
static void rom_pbus_xpd_tx_off(void){}
static void rom_pbus_xpd_rx_on(int on){}
static void rom_set_rxclk_en(int on){}
static void esp_rom_delay_us(unsigned us){}
#define BURST_SERIAL_UART 1
static int burst_serial_port(void){return BURST_SERIAL_UART;}
static uint32_t samples[16384];
static size_t sent;
static bool burst_serial_send(const void *data,size_t size){sent=size;return true;}
static unsigned rtc_clk_xtal_freq_get(void){return xtal;}
void set_chanfreq(unsigned n,unsigned mode){assert(mode==0);channel=n;pll=n;pll_offset=0;tunes++;}
void rom_set_rf_freq_offset(unsigned c,unsigned n,int offset){pll_offset=offset;crystal=c;pll=n;}
static void vTaskDelay(unsigned n){}
static bool acquire_iq(unsigned n,unsigned source,unsigned clock,unsigned *us){captures++;clock_code=clock;*us=1;return true;}
'''
        checks=r'''
static void run(const char *text){char line[128];snprintf(line,sizeof(line),"%s",text);command(line);}
int main(void){
 run("INFO");assert(!strcmp(response,"ESP32SDR 6 burst 16380\n"));
 run("CAPS");assert(strstr(response,"RXLIMITS"));assert(strstr(response,"LPFANA"));assert(!strstr(response,"DUALSERIAL"));
 run("LIMITS?");assert(strstr(response,"[0,72,1]"));assert(strstr(response,"\"bandwidth\":[12,67,1,0]"));
 run("TRANSPORT?");assert(!strcmp(response,"TRANSPORT UART 2000000\n"));
 run("SYNC 18446744073709551615");assert(!strcmp(response,"SYNC 18446744073709551615\n"));
 run("RANGE?");assert(!strcmp(response,"RANGE 100 6000 1\n"));
 run("CAPS");assert(strstr(response,"TUNEEXT"));
 for(unsigned f=100;f<=6000;f++) {
   char cmd[32];snprintf(cmd,sizeof(cmd),"FREQ %u",f);run(cmd);
   assert(!strcmp(response,"OK\n"));
   unsigned khz=f*((f>=1842 && f<2210)?1200u:1000u);
   int64_t error=((int64_t)pll*1024+pll_offset)*1000-(int64_t)khz*1024;
   assert(error>=-500 && error<=500);
   assert(channel==(((f>=2412 && f<=2472 && (f-2412)%5==0)||f==2484)?f:2412));
 }
 run("FREQ 2413");assert(crystal==0);
 xtal=26;run("FREQ 2413");assert(crystal==1);
 xtal=24;run("FREQ 2413");assert(crystal==2);
 run("FREQ 2472");assert(channel==2472 && pll==2472);
 run("FREQ 2413");run("FREQ 2412");assert(channel==2412 && pll==2412);
 unsigned before=tunes;run("FREQ 2412");assert(tunes==before+1);
 run("GAIN MANUAL 72");assert(!hardware_agc&&gain_code==72);run("GAIN?");assert(!strcmp(response,"GAIN MANUAL 72 0 72 1\n"));
 run("BANDWIDTH 12");assert(rx_filter==127);run("BANDWIDTH 67");assert(rx_filter==8);run("BANDWIDTH 0");assert(rx_filter==0);
 run("LPF 63");assert(rx_filter==63);run("LPF AUTO");assert(rx_filter==-1);
 run("GAIN HARDWARE");assert(hardware_agc);run("GAIN?");assert(strstr(response,"HARDWARE -1"));
 run("CAP16 16380 0");assert(captures==1&&sent==16380*2&&clock_code==1);
 run("CAP20 16379 1");assert(captures==2&&sent==(16379*20+7)/8&&clock_code==2);
 run("CAP16 256 6");assert(captures==3&&sent==512&&clock_code==0);
 run("RXRUN 16380 0 2 20");assert(captures==5&&sent==16380*20/8&&!strcmp(response,"END\n"));
 const char *bad[]={"CAP16 16381 0","CAP20 255 0","CAP16 4096 2","CAP16 4096 0 junk","RXRUN 16380 0 1001 20","GAIN MANUAL 73","GAIN MANUAL -1","GAIN SOFTWARE","FREQ 99","FREQ 6001","FREQ -1","FREQ 2413 junk","FREQ 6001","BANDWIDTH 11","BANDWIDTH 68","LPF 128","TX16 256 0","CW START"};
 for(unsigned j=0;j<sizeof(bad)/sizeof(bad[0]);j++){run(bad[j]);assert(!strncmp(response,"ERR",3));}
 assert(captures==5);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'check.c').write_text(stub+handler+checks)
            (p/'esp_rom_crc.h').write_text('static uint32_t esp_rom_crc32_le(uint32_t c,const uint8_t *d,size_t n){return c;}\n')
            subprocess.run(['cc','-std=c11','-I'+tmp,'-I'+str(Path(__file__).resolve().parents[1]/'main/common'),str(p/'check.c'),'-o',str(p/'check')],check=True)
            subprocess.run([str(p/'check')],check=True)
