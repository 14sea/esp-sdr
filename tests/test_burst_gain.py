from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class BurstGain(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_hardware_default_manual_selection_and_removed_software_command(self):
        header = Path(__file__).resolve().parents[1] / 'main/common/burst_gain.h'
        code = r'''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static unsigned forced, selected, mock_maximum=76;
static char response[128];
void force_rx_gain(unsigned force, unsigned gain, unsigned unused) { forced=force; selected=gain; }
static void reply(const char *s) { snprintf(response,sizeof(response),"%s",s); }
#define REG_READ(address) ((forced << 23)|(mock_maximum << 8))
#include "HEADER"
int main(void) {
 gain_apply(); assert(forced==0);
 assert(gain_command("GAIN?")); assert(strstr(response,"GAIN HARDWARE -1"));
 assert(gain_command("GAIN MANUAL 23")); assert(forced==1 && selected==23);
 assert(gain_command("GAIN?")); assert(strstr(response,"GAIN MANUAL 23"));
 assert(!gain_command("GAIN AUTO")); assert(forced==1 && selected==23);
 assert(!gain_command("GAIN MANUAL 99")); assert(selected==23);
 assert(gain_command("GAIN MANUAL 76")); assert(selected==76);
 assert(!gain_command("GAIN MANUAL 77")); assert(selected==76);
 mock_maximum=71; assert(!gain_command("GAIN MANUAL 72"));
 assert(gain_command("GAIN HARDWARE")); assert(forced==0);
 assert(gain_command("GAIN?")); assert(strstr(response,"GAIN HARDWARE -1"));
 return 0;
}
'''.replace('HEADER',str(header))
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'gain.c';source.write_text(code)
            for c5 in [0,1]:
                binary=Path(temp)/('gain-'+str(c5))
                subprocess.run(['cc','-std=c11',f'-DCONFIG_IDF_TARGET_ESP32C5={c5}',str(source),'-o',str(binary)],check=True)
                subprocess.run([str(binary)],check=True)
