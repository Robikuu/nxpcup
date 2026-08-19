#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include "vectors.h"

void esp_printf(const char *fmt, ...) { (void)fmt; }

int main(void) {
    // Vector format: [x0, y0, x1, y1, idx, flags]
    // Vector 1: Forward line (x0=20, y0=50, x1=20, y1=10) -> dy=40, dx=0, slope=0.0 (KEEP)
    // Vector 2: Crossing tape (x0=5, y0=35, x1=35, y1=0) -> dy=35, dx=30, slope=0.857 > 0.75 (DISCARD slope)
    // Vector 3: Horizontal tape (x0=10, y0=20, x1=60, y1=20) -> dy=0 < 4.0 (DISCARD dy)
    // Vector 4: Short vertical jitter (x0=30, y0=23, x1=30, y1=20) -> dy=3 < 4.0 (DISCARD dy)
    // Vector 5: Short total length (x0=40, y0=25, x1=43, y1=21) -> dy=4, dx=3, len=5.0 < 8.0 (DISCARD len)
    // Vector 6: Slanted line within slope limit (x0=55, y0=45, x1=65, y1=25) -> dy=20, dx=10, slope=0.5 <= 0.75, len=22.36 >= 8.0 (KEEP)
    // Vector 7: Inverted coords (y0 < y1) valid (x0=15, y0=15, x1=15, y1=45) -> dy=30, dx=0, len=30 >= 8.0 (KEEP)
    uint16_t vecs[7 * 6] = {
        20, 50, 20, 10, 0, 0,
        5,  35, 35, 0,  1, 0,
        10, 20, 60, 20, 2, 0,
        30, 23, 30, 20, 3, 0,
        40, 25, 43, 21, 4, 0,
        55, 45, 65, 25, 5, 0,
        15, 15, 15, 45, 6, 0
    };

    size_t kept = filter_intersection_vectors(vecs, 7);
    assert(kept == 3);
    assert(vecs[0] == 20 && vecs[1] == 50 && vecs[2] == 20 && vecs[3] == 10);
    assert(vecs[6] == 55 && vecs[7] == 45 && vecs[8] == 65 && vecs[9] == 25);
    assert(vecs[12] == 15 && vecs[13] == 15 && vecs[14] == 15 && vecs[15] == 45);

    // Empty vector set test
    assert(filter_intersection_vectors(NULL, 0) == 0);

    printf("Intersection filtering tests PASSED!\n");
    return 0;
}
