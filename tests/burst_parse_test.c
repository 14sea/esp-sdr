#include <assert.h>
#include <string.h>
#include "burst_parse.h"

int main(void) {
    unsigned v[6];
    assert(burst_parse_uints("1000 64 8 6", v, 6) == 4 && v[0] == 1000 && v[3] == 6);
    assert(burst_parse_uints("  0 64  8 6 7 2 ", v, 6) == 6 && v[4] == 7 && v[5] == 2);
    assert(burst_parse_uints("", v, 6) == 0);
    assert(burst_parse_uints("1000 64 8 6 trailing", v, 6) == -1);
    assert(burst_parse_uints("1000 64x 8 6", v, 6) == -1);
    assert(burst_parse_uints("1 2 3 4 5 6 7", v, 6) == -1);
    assert(burst_parse_uints("-1", v, 6) == -1);
    assert(burst_parse_uints("4294967295", v, 1) == 1 && v[0] == 4294967295u);
    assert(burst_parse_uints("4294967296", v, 1) == -1);
    assert(burst_parse_uints("2147483649", v, 1) == 1);
    int x;
    assert(burst_parse_int("-1000", -1000, 1000, &x) && x == -1000);
    assert(burst_parse_int("+999", -1000, 1000, &x) && x == 999);
    assert(burst_parse_int(" 0", -1000, 1000, &x) && x == 0);
    assert(!burst_parse_int("1001", -1000, 1000, &x));
    assert(!burst_parse_int("2147483647", -1000, 1000, &x));
    assert(!burst_parse_int("99999999999", -1000, 1000, &x));
    assert(!burst_parse_int("- 5", -1000, 1000, &x));
    assert(!burst_parse_int("5 6", -1000, 1000, &x));
    assert(!burst_parse_int("", -1000, 1000, &x));
    return 0;
}
