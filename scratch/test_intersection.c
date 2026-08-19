#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include "vectors.h"

void esp_printf(const char *fmt, ...) { (void)fmt; }

int main(void) {
    // Vector format: [x0, y0, x1, y1, idx, flags]
    // Vector 1: Forward line (x0=20, y0=50, x1=20, y1=10) -> dy=40, dx=0, slope=0.0 (KEEP)
    // Vector 2: Crossing tape (x0=5, y0=35, x1=35, y1=0) -> dy=35, dx=30, slope=0.857 > 0.75 (DISCARD)
    // Vector 3: Horizontal tape (x0=10, y0=20, x1=60, y1=20) -> dy=0 (DISCARD)
    uint16_t vecs[3 * 6] = {
        20, 50, 20, 10, 0, 0,
        5,  35, 35, 0,  1, 0,
        10, 20, 60, 20, 2, 0
    };

    size_t kept = filter_intersection_vectors(vecs, 3);
    assert(kept == 1);
    assert(vecs[0] == 20 && vecs[1] == 50 && vecs[2] == 20 && vecs[3] == 10);
    printf("Intersection filtering test PASSED!\n");
    return 0;
}
