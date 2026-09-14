#ifndef VECTORS_H
#define VECTORS_H
#include <stdint.h>
#include <stddef.h>

typedef struct {
    float   cte;
    float   heading_deg;
    uint8_t found;
} steering_result_t;

void vector_filtering(uint16_t *vectors, size_t *num_vectors);
steering_result_t compute_steering_error(const uint16_t *vectors, size_t num_vectors);

#endif // VECTORS_H