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
 assert(rx_bandwidth_dcap(RX_BANDWIDTH_MAX)==0);
 assert(rx_bandwidth_dcap(RX_BANDWIDTH_MIN)==60);
 for(unsigned mhz=RX_BANDWIDTH_MIN+1;mhz<=RX_BANDWIDTH_MAX;mhz++)
   assert(rx_bandwidth_dcap(mhz)<=rx_bandwidth_dcap(mhz-1));
#if CONFIG_IDF_TARGET_ESP32C6
 assert(rx_bandwidth_dcap(48)==4);assert(rx_bandwidth_dcap(39)==8);
 assert(rx_bandwidth_dcap(28)==16);assert(rx_bandwidth_dcap(20)==32);
 assert(rx_bandwidth_dcap(15)==48);assert(rx_bandwidth_dcap(12)==60);
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
        for chip in ['ESP32C6','ESP32C61','ESP32S3','ESP32S31']:
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
