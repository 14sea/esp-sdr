"""Exercise production snapshot accumulation/encoding with deterministic FFT output."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SnapshotSpectrum(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_mean_max_bin_order_and_frame_counts(self):
        source = (ROOT / 'main/common/spectrum.c').read_text()
        sdk_headers = ['esp_cpu.h', 'dsps_fft2r.h', 'esp_rom_crc.h', 'esp_timer.h',
                       'freertos/FreeRTOS.h', 'freertos/task.h']
        for header in sdk_headers:
            source = source.replace(f'#include "{header}"', '')
        stub = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "spectrum_stats.h"
#include "burst_serial.h"
#define ESP_OK 0
#define CONFIG_IDF_TARGET_ESP32C3 1
unsigned spectrum_dc_mode;
static unsigned transforms,frames,wanted_units,wanted_n,wanted_detector;
static uint32_t words[2048];
static uint32_t esp_cpu_get_cycle_count(void){return 0;}
static int64_t esp_timer_get_time(void){static int64_t t;return t+=1000;}
static void vTaskDelay(unsigned ticks){}
static unsigned dsps_fft2r_init_sc16(int16_t *table,unsigned n){return ESP_OK;}
static void dsps_fft2r_sc16_ansi(int16_t *x,unsigned n){
    for(unsigned j=0;j<n;j++){x[2*j]=(j%97+1)*(transforms+1);x[2*j+1]=0;}
    transforms++;
}
static uint32_t esp_rom_crc32_le(uint32_t c,const void *p,unsigned n){return 0;}
burst_serial_port_t burst_serial_port(void){return BURST_SERIAL_USB;}
bool burst_serial_stop_requested(void){return false;}
void spectrum_stats_init(spectrum_stats_t *s){memset(s,0,sizeof(*s));}
void spectrum_stats_emit(spectrum_stats_t *s,unsigned n,unsigned fs,uint32_t ffts,
 unsigned abandoned,uint32_t drops,uint32_t late,unsigned queue,bool (*send)(const void*,size_t)){}
'''
        check = r'''
static bool acquire(unsigned n,unsigned rate,const uint32_t **p,unsigned *us){*p=words;*us=1;return true;}
bool burst_serial_send(const void *data,size_t size){
 const uint8_t *b=data;
 if(size<4||memcmp(b,"SPC1",4))return true;
 frames++;assert(size==wanted_n+32);assert(b[20]==wanted_units&&b[21]==0);
 assert((b[22]&1)==wanted_detector);assert(b[27]==2);
 unsigned pairs=b[16]|b[17]<<8|b[18]<<16|b[19]<<24;
 assert(pairs==wanted_n*wanted_units);
 for(unsigned j=1;j<wanted_n;j++){
  if(j==wanted_n/2||j==wanted_n-1)continue; /* DC correction neighbors */
  double p=0;
  for(unsigned u=1;u<=wanted_units;u++){
   double v=(j%97+1)*u;v*=v;
   p=wanted_detector?fmax(p,v):p+v;
  }
  if(!wanted_detector)p/=wanted_units;
  int expected=(int)lrint(20*log10(p));if(expected>255)expected=255;
  assert(abs((int)b[28+reverse(j,b[26])]-expected)<=1);
 }
 return true;
}
int main(void){
 for(wanted_n=256;wanted_n<=2048;wanted_n*=2)
 for(wanted_units=1;wanted_units<=8;wanted_units++)
 for(wanted_detector=0;wanted_detector<=1;wanted_detector++){
  transforms=frames=0;char command[64];
  snprintf(command,sizeof(command),"SPEC 1 1 %u %u 0 %u",wanted_units,wanted_detector,wanted_n);
  assert(spectrum_command(command,2412,acquire));
  assert(frames==1&&transforms==wanted_units);
 }
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'snapshot.c'
            src.write_text('#include <stdlib.h>\n' + stub + source + check)
            exe = Path(tmp) / 'snapshot'
            subprocess.run(['cc', '-std=gnu11', '-O2', '-fsanitize=undefined',
                            '-I' + str(ROOT / 'main/common'), str(src), '-lm', '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
