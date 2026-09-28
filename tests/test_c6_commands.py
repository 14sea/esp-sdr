"""Run the shared production parser with C6 capabilities and hardware stubs."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

class C6Commands(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_c6(self):
        source=(Path(__file__).resolve().parents[1]/'main/c5_c61_main.c').read_text()
        handler=source[source.index('static void handle_command(char *line) {'):]
        stub=r'''
#include <assert.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#define BURST_SERIAL_UART 1
#define IQ_WORDS 16380u
#define RX_FILTER_REG 4u
#include "rx_bandwidth.h"
#define CONFIG_ESP_SDR_UART_ENABLED 1
#if CONFIG_IDF_TARGET_ESP32C61
#define BURST_ID "C61SDR"
#else
#define BURST_ID "C6SDR"
#endif
static unsigned frequency_mhz,captures,last_format,last_samples;
static int rx_filter,rx_analog_filter;
static bool rx_ready;
static char response[128];
static int burst_serial_port(void) { return 1; }
static unsigned burst_serial_baud(void) { return 2000000; }
static void reply(const char *s) { snprintf(response,sizeof(response),"%s",s); }
static bool gain_command(const char *s) { return false; }
static bool limits_command(const char *s) { return false; }
static bool capture(unsigned n,unsigned d,unsigned f) { ++captures;last_samples=n;last_format=f;return true; }
static void vTaskDelay(int ticks) {}
static unsigned phy_chip_i2c_readReg(unsigned a,unsigned b,unsigned c) { return 4; }
static void prepare_rx(void) { rx_ready=true; }
#define REG_READ(a) 0u
static bool frequency_valid(unsigned f) {
#if CONFIG_IDF_TARGET_ESP32C61
return f>=2400 && f<=2500;
#else
return f>=2100 && f<=2800;
#endif
}
'''
        check=r'''
static void command(const char *s) { char line[128];snprintf(line,sizeof(line),"%s",s);handle_command(line); }
int main(void) {
 command("INFO");assert(!strcmp(response,BURST_ID " 6 burst 16380\n"));
 command("CAPS");assert(strstr(response,"SERIALLEASE"));assert(strstr(response,"IQ8"));
 assert(!!strstr(response,"DUALSERIAL")==1);
 command("SYNC 987654321");assert(!strcmp(response,"SYNC 987654321\n"));
 command("CAP16 16380 0");assert(captures==1 && last_samples==16380 && last_format==16);
 command("CAP20 257 0");assert(captures==2 && last_samples==257 && last_format==20);
 command("RXRUN 256 0 2 20");assert(captures==4 && !strcmp(response,"END\n"));
 const char *bad[]={"CAP16 16381 0","CAP20 255 0","CAP20 16380 6","CAP20 256 0 junk",
 "RXRUN 256 0 0 20","RXRUN 256 0 1001 20","RXRUN 256 0 2 32","FREQ 5180 junk",
 "FREQ 2399.5","GAIN AUTO","TX20 256 1000000 0","CW START","REPLAY20 256 40000000 0"};
 for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++){command(bad[i]);assert(!strcmp(response,"ERR command\n"));}
 assert(captures==4);
 command("FREQ 2412");assert(frequency_mhz==2412 && rx_ready);
 command("FREQ 5180");assert(frequency_mhz==2412);
 for(unsigned d=1;d<=5;d++){char c[40];snprintf(c,sizeof(c),"CAP20 256 %u",d);command(c);assert(!strcmp(response,"ERR command\n"));}
 command("LPF 12");assert(!strcmp(response,"ERR command\n"));
 command("ALPF 63");assert(rx_analog_filter==63);
 command("ALPF AUTO");assert(rx_analog_filter==-1);
 command("ALPF 64");assert(!strcmp(response,"ERR command\n"));
 command("RANGE?");assert(!strcmp(response,"RANGE 2100 2800 1\n"));
 command("CAPS");assert(strstr(response,"TUNEEXT"));
 const unsigned extended[]={2100,2300,2402,2426,2480,2500,2800};
 for(unsigned i=0;i<sizeof(extended)/sizeof(extended[0]);i++){
   char c[40];snprintf(c,sizeof(c),"FREQ %u",extended[i]);command(c);
   assert(!strcmp(response,"OK\n") && frequency_mhz==extended[i] && rx_ready);
 }
 command("FREQ 2099");assert(!strcmp(response,"ERR command\n"));
 command("FREQ 2801");assert(!strcmp(response,"ERR command\n"));
 command("BANDWIDTH 0");assert(rx_analog_filter==0);
 command("BANDWIDTH 54");assert(rx_analog_filter==0);
 command("BANDWIDTH 12");assert(rx_analog_filter==60);
 command("BANDWIDTH 20");assert(rx_analog_filter==32);
 command("BANDWIDTH 11");assert(!strcmp(response,"ERR command\n"));
 command("BANDWIDTH 55");assert(!strcmp(response,"ERR command\n"));
 command("BANDWIDTH 1");assert(!strcmp(response,"ERR command\n"));
 command("BANDWIDTH 21 junk");assert(!strcmp(response,"ERR command\n"));

 command("RELEASE");assert(!strcmp(response,"OK\n"));
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'commands.c';path.write_text(stub+handler+check)
            for c61 in [0]:
                binary=Path(tmp)/f'commands-{c61}'
                subprocess.run(['cc','-std=c11','-DCONFIG_IDF_TARGET_ESP32C6=1','-I'+str(Path(__file__).resolve().parents[1]/'main'),f'-DCONFIG_IDF_TARGET_ESP32C61={c61}','-DCONFIG_IDF_TARGET_ESP32C5=0',str(path),'-o',str(binary)],check=True)
                subprocess.run([str(binary)],check=True)
