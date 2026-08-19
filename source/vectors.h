#ifndef VECTORS_H
#define VECTORS_H

#include <stddef.h>
#include <stdint.h>

#define PIXY_CENTER_X           39.0f
#define LOOKAHEAD_Y             22
#define INTERSECTION_MAX_SLOPE  0.75f

typedef struct {
    float   cte;          /* lateral offset from center in pixels (+ = right of center) */
    float   heading;      /* slope: lateral px / forward px (+ = tilting right)          */
    uint8_t lines_found;  /* 0 = lost, 1 = single line tracked, 2 = dual lane boundaries */
} lane_result_t;

void          reset_lane_tracker(void);
lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors);
size_t        filter_intersection_vectors(uint16_t *vectors, size_t num_vectors);
void          print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index);

#endif /* VECTORS_H */