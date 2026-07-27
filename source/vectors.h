#ifndef VECTORS_H
#define VECTORS_H

#include <math.h>
#include <stdint.h>

void pixel_to_ground(float u, float v, float *X, float *Y);
void print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index);
void populate_vectors(uint16_t *vectors, size_t num_vectors);

typedef struct {
    float x0_ground, y0_ground, x1_ground, y1_ground; // real-world cm, not pixels
    float angle_deg;
    uint8_t index;
} track_vector_t;

#endif