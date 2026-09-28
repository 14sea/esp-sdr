from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

@unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
class RxControls(unittest.TestCase):
    def compile_run(self, source, flags=()):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'check.c';path.write_text(source)
            exe=Path(tmp)/'check'
            subprocess.run(['cc','-std=c11','-I'+str(ROOT/'main'),*flags,str(path),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_bandwidth_curves_and_interpolation(self):
        source=r'''
#include <assert.h>
#include "rx_bandwidth.h"
int main(void) {
 assert(rx_bandwidth_dcap(0)==0);
#if CONFIG_IDF_TARGET_ESP32S2
 assert(rx_bandwidth_dcap(RX_BANDWIDTH_MAX)==8);
 assert(rx_bandwidth_dcap(RX_BANDWIDTH_MIN)==63);
#elif CONFIG_IDF_TARGET_ESP32
 assert(rx_bandwidth_dcap(RX_BANDWIDTH_MAX)==8);
 assert(rx_bandwidth_dcap(RX_BANDWIDTH_MIN)==127);
#else
 assert(rx_bandwidth_dcap(RX_BANDWIDTH_MAX)==0);
 assert(rx_bandwidth_dcap(RX_BANDWIDTH_MIN)==60);
#endif
 for(unsigned mhz=RX_BANDWIDTH_MIN+1;mhz<=RX_BANDWIDTH_MAX;mhz++)
   assert(rx_bandwidth_dcap(mhz)<=rx_bandwidth_dcap(mhz-1));
#if CONFIG_IDF_TARGET_ESP32
 assert(rx_bandwidth_dcap(48)==16);assert(rx_bandwidth_dcap(20)==64);
 assert(rx_bandwidth_dcap(32)==32);assert(rx_bandwidth_dcap(15)==96);
#elif CONFIG_IDF_TARGET_ESP32C5
 assert(RX_BANDWIDTH_MIN==11 && RX_BANDWIDTH_MAX==23);
 assert(rx_bandwidth_dcap(20)==12);assert(rx_bandwidth_dcap(16)==24);
 assert(rx_bandwidth_dcap(19)==14);assert(rx_bandwidth_dcap(12)==48);
#elif CONFIG_IDF_TARGET_ESP32C6
 assert(rx_bandwidth_dcap(48)==4);assert(rx_bandwidth_dcap(39)==8);
 assert(rx_bandwidth_dcap(28)==16);assert(rx_bandwidth_dcap(20)==32);
 assert(rx_bandwidth_dcap(15)==48);assert(rx_bandwidth_dcap(12)==60);
#elif CONFIG_IDF_TARGET_ESP32S2
 assert(RX_BANDWIDTH_MIN==15 && RX_BANDWIDTH_MAX==60);
 assert(rx_bandwidth_dcap(41)==16);assert(rx_bandwidth_dcap(26)==32);
#elif CONFIG_IDF_TARGET_ESP32S3
 assert(rx_bandwidth_dcap(51)==4);assert(rx_bandwidth_dcap(33)==16);
 assert(rx_bandwidth_dcap(25)==24);assert(rx_bandwidth_dcap(21)==32);
#elif CONFIG_IDF_TARGET_ESP32S31
 assert(rx_bandwidth_dcap(21)==28);
#else
 assert(rx_bandwidth_dcap(21)==24);assert(rx_bandwidth_dcap(36)==8);
 assert(rx_bandwidth_dcap(33)==12);assert(rx_bandwidth_dcap(14)==54);
#endif
 return 0;
}
'''
        for chip in ['ESP32','ESP32C5','ESP32C6','ESP32C61','ESP32S2','ESP32S3','ESP32S31']:
            self.compile_run(source,[f'-DCONFIG_IDF_TARGET_{chip}=1'])

    def test_c61_mirror_and_agc_restore(self):
        self.compile_run(r'''
#include <assert.h>
#include <stdint.h>
static uint32_t hardware[160][3];
void __real_phy_write_gain_mem(uint32_t a,uint32_t b,uint32_t c,uint32_t index) {
 assert(index<160);hardware[index][0]=a;hardware[index][1]=b;hardware[index][2]=c;
}
#include "burst_gain_table.c"
int main(void) {
 __wrap_phy_write_gain_mem(10,20,30,20);
 __wrap_phy_write_gain_mem(40,50,60,100);
 __wrap_phy_write_gain_mem(70,80,90,76);
 burst_gain_mirror(20);assert(hardware[100][0]==10);
 burst_gain_mirror(76);assert(hardware[100][0]==40);
 /* An uncalibrated high-table slot still needs a forced-gain mirror. */
 assert(hardware[156][0]==70 && hardware[156][2]==90);
 burst_gain_mirror(-1);assert(mirrored==-1);
 burst_gain_mirror(20);burst_gain_mirror(-1);assert(hardware[100][2]==60);
 burst_gain_mirror(80);assert(mirrored==-1);
 return 0;
}
''')

    def test_original_esp32_filter_restores_calibration(self):
        source=(ROOT/'main/esp32_main.c').read_text()
        functions=source[source.index('static int rx_filter'):source.index('extern void rom_pbus_workmode')]
        self.compile_run(r'''
#include <assert.h>
static unsigned values[2]={0xd0,0xd2},writes;
unsigned rom_chip_i2c_readReg(unsigned b,unsigned h,unsigned r){
 assert(b==0x67 && h==1 && (r==1 || r==2));return values[r-1];
}
void rom_chip_i2c_writeReg(unsigned b,unsigned h,unsigned r,unsigned v){
 assert(b==0x67 && h==1 && (r==1 || r==2));values[r-1]=v;writes++;
}
void esp_rom_delay_us(unsigned us){}
''' + functions + r'''
int main(void){
 filter_apply();filter_restore();assert(writes==0);
 rx_filter=0;filter_apply();assert(values[0]==0x80 && values[1]==0x80);
 filter_restore();assert(values[0]==0xd0 && values[1]==0xd2);
 rx_filter=127;filter_apply();assert(values[0]==0xff && values[1]==0xff);
 filter_restore();assert(values[0]==0xd0 && values[1]==0xd2);
 /* Retuning may change calibration; take a fresh backup each capture. */
 values[0]=0xc4;values[1]=0xc6;rx_filter=16;filter_apply();
 assert(values[0]==0x90 && values[1]==0x90);filter_restore();
 assert(values[0]==0xc4 && values[1]==0xc6);
}
''')
