#ifndef VECTORS_H
#define VECTORS_H

#include <math.h>
#include <stdint.h>

#define H11  0.007548889315f
#define H12  0.0002899088594f
#define H13 -1.290254073f
#define H21  8.625606481e-05f
#define H22 -0.004575319953f
#define H23  1.733411797f
#define H31 -0.0006622785396f
#define H32  0.02125916896f
#define H33  1.0f

void pixel_to_ground(float u, float v, float *X, float *Y);
void print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index);
void populate_vectors(uint16_t *vectors, size_t num_vectors);

typedef struct {
    float x0_ground, y0_ground, x1_ground, y1_ground; // real-world cm, not pixels
    float angle_deg;
    uint8_t index;
} track_vector_t;

#endif