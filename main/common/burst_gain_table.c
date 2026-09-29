/* Keep calibrated C61 gain words, including the high-table originals.
 * Restore calibrated high-table slots on AGC; slots above the high-table
 * maximum are unused by AGC but must still be mirrored for forced gain.
 * Only 2080 bytes; no sample buffers or heap allocations. */
#include <stdint.h>
#include <stdbool.h>
static uint32_t words[160][3];
static bool valid[160];
static int mirrored=-1;
extern void __real_phy_write_gain_mem(uint32_t,uint32_t,uint32_t,uint32_t);
void __wrap_phy_write_gain_mem(uint32_t a,uint32_t b,uint32_t c,uint32_t index) {
    if(index<160) { words[index][0]=a;words[index][1]=b;words[index][2]=c;valid[index]=true; }
    __real_phy_write_gain_mem(a,b,c,index);
}
void burst_gain_mirror(int index) {
    if(mirrored>=0) {
        unsigned high=(unsigned)mirrored+80;
        if(valid[high])__real_phy_write_gain_mem(words[high][0],words[high][1],words[high][2],high);
        mirrored=-1;
    }
    if(index>=0 && index<80 && valid[index]) {
        __real_phy_write_gain_mem(words[index][0],words[index][1],words[index][2],index+80);
        mirrored=index;
    }
}
