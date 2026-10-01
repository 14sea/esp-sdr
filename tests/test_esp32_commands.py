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
        handler=source[source.index('extern void set_chanfreq('):source.index('void app_main(')]
        stub=r'''
#include <assert.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#define CONFIG_IDF_TARGET_ESP32 1
#include "rx_bandwidth.h"
#include "rx_tuning.h"
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
static bool ring_test(const char *s){return false;}
static void reply(const char *fmt,...) {va_list a;va_start(a,fmt);vsnprintf(response,sizeof(response),fmt,a);va_end(a);}
static unsigned burst_serial_baud(void){return 2000000;}
static void apply_gain(void){}
static void prepare_rx(void){}
static unsigned rtc_clk_xtal_freq_get(void){return xtal;}
void set_chanfreq(unsigned n,unsigned mode){assert(mode==0);channel=n;pll=n;tunes++;}
void rom_set_rf_freq_offset(unsigned c,unsigned n,int offset){assert(offset==0);crystal=c;pll=n;}
static void vTaskDelay(unsigned n){}
static bool capture(unsigned n,unsigned source,unsigned clock,unsigned format){captures++;bits=format;clock_code=clock;return true;}
'''
        checks=r'''
int main(void){
 command("INFO");assert(!strcmp(response,"ESP32SDR 6 burst 16380\n"));
 command("CAPS");assert(strstr(response,"RXLIMITS"));assert(strstr(response,"LPFANA"));assert(!strstr(response,"DUALSERIAL"));
 command("LIMITS?");assert(strstr(response,"[0,72,1]"));assert(strstr(response,"\"bandwidth\":[12,67,1,0]"));
 command("TRANSPORT?");assert(!strcmp(response,"TRANSPORT UART 2000000\n"));
 command("SYNC 18446744073709551615");assert(!strcmp(response,"SYNC 18446744073709551615\n"));
 command("RANGE?");assert(!strcmp(response,"RANGE 100 6000 1\n"));
 command("CAPS");assert(strstr(response,"TUNEEXT"));
 for(unsigned f=100;f<=6000;f++) {
   char cmd[32];snprintf(cmd,sizeof(cmd),"FREQ %u",f);command(cmd);
   assert(!strcmp(response,"OK\n") && pll==f);
   assert(channel==(((f>=2412 && f<=2472 && (f-2412)%5==0)||f==2484)?f:2412));
 }
 command("FREQ 2413");assert(crystal==0);
 xtal=26;command("FREQ 2413");assert(crystal==1);
 xtal=24;command("FREQ 2413");assert(crystal==2);
 command("FREQ 2472");assert(channel==2472 && pll==2472);
 command("FREQ 2413");command("FREQ 2412");assert(channel==2412 && pll==2412);
 unsigned before=tunes;command("FREQ 2412");assert(tunes==before+1);
 command("GAIN MANUAL 72");assert(!hardware_agc&&gain_code==72);command("GAIN?");assert(!strcmp(response,"GAIN MANUAL 72 0 72 1\n"));
 command("BANDWIDTH 12");assert(rx_filter==127);command("BANDWIDTH 67");assert(rx_filter==8);command("BANDWIDTH 0");assert(rx_filter==0);
 command("LPF 63");assert(rx_filter==63);command("LPF AUTO");assert(rx_filter==-1);
 command("GAIN HARDWARE");assert(hardware_agc);command("GAIN?");assert(strstr(response,"HARDWARE -1"));
 command("CAP16 16380 0");assert(captures==1&&bits==8&&clock_code==1);
 command("CAP20 16379 1");assert(captures==2&&bits==10&&clock_code==2);
 command("CAP16 256 6");assert(captures==3&&bits==8&&clock_code==0);
 command("RXRUN 16380 0 2 20");assert(captures==5&&bits==10&&!strcmp(response,"END\n"));
 const char *bad[]={"CAP16 16381 0","CAP20 255 0","CAP16 4096 2","CAP16 4096 0 junk","RXRUN 16380 0 1001 20","GAIN MANUAL 73","GAIN MANUAL -1","GAIN SOFTWARE","FREQ 99","FREQ 6001","FREQ -1","FREQ 2413 junk","FREQ 6001","BANDWIDTH 11","BANDWIDTH 68","LPF 128","TX16 256 0","CW START"};
 for(unsigned j=0;j<sizeof(bad)/sizeof(bad[0]);j++){command(bad[j]);assert(!strncmp(response,"ERR",3));}
 assert(captures==5);command("RELEASE");assert(!strcmp(response,"OK\n"));
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'check.c').write_text(stub+handler+checks)
            subprocess.run(['cc','-std=c11','-I'+str(Path(__file__).resolve().parents[1]/'main/common'),str(p/'check.c'),'-o',str(p/'check')],check=True)
            subprocess.run([str(p/'check')],check=True)
