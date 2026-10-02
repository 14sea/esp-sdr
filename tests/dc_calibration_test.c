#include "stream.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static unsigned codes[4][2], writes, observations, fail_at;
static float offset[2], response[4];
void phy_pbus_debugmode(void) {}
unsigned phy_pbus_rd(unsigned block, unsigned bank) { return codes[block][bank - 1]; }
void phy_pbus_force_test(unsigned block, unsigned bank, unsigned value) {
    assert(block < 4 && bank >= 1 && bank <= 2 && value <= 511);
    codes[block][bank - 1] = value;
    writes++;
}
void vTaskDelay(unsigned ticks) { (void)ticks; }
static bool measure(float mean[2]) {
    if (++observations == fail_at)
        return false;
    float i = (int)codes[2][1] - 256, q = (int)codes[3][1] - 256;
    mean[0] = offset[0] + response[0] * i + response[1] * q;
    mean[1] = offset[1] + response[2] * i + response[3] * q;
    return true;
}
static void setup(float i, float q, const float matrix[4]) {
    for (unsigned b = 0; b < 4; b++)
        for (unsigned n = 0; n < 2; n++)
            codes[b][n] = 256;
    offset[0] = i;
    offset[1] = q;
    memcpy(response, matrix, sizeof(response));
    writes = observations = fail_at = 0;
}
int main(void) {
    const float plants[][4] = {
        {.2f, 0, 0, .3f}, {0, .4f, -.3f, 0}, {.2f, .1f, -.1f, .2f}, {-.2f, 0, 0, -.4f}};
    for (unsigned p = 0; p < 4; p++) {
        setup(9, -7, plants[p]);
        receiver_dc_calibrate(measure);
        float m[2];
        assert(measure(m));
        assert(hypotf(m[0], m[1]) < .51f);
        assert(receiver_dc_steps <= 15);
        // The actuator must not change the RF/baseband gain controls.
        for (unsigned b = 0; b < 2; b++)
            for (unsigned n = 0; n < 2; n++)
                assert(codes[b][n] == 256);
    }
    const float flat[] = {.001f, 0, 0, .001f};
    setup(10, -10, flat);
    receiver_dc_calibrate(measure);
    assert(codes[2][1] == 256 && codes[3][1] == 256);
    setup(0, 0, plants[0]);
    receiver_dc_calibrate(measure);
    assert(writes == 0);
    setup(10, -10, plants[0]);
    fail_at = 1;
    receiver_dc_calibrate(measure);
    assert(writes == 0);
    setup(10, -10, plants[0]);
    fail_at = 2;
    receiver_dc_calibrate(measure);
    assert(codes[2][1] == 256 && codes[3][1] == 256);
    setup(200, -200, plants[0]);
    receiver_dc_calibrate(measure);
    assert(codes[2][1] >= 160 && codes[2][1] <= 352);
    assert(codes[3][1] >= 160 && codes[3][1] <= 352);
    puts("Analog calibration: cross-coupled/inverted plants, bounds, and failure rollback passed");
}
