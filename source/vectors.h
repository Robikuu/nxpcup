#ifndef VECTORS_H
#define VECTORS_H

#include <math.h>
#include <stdint.h>

///DO NOT CHANGE EVER!!!!!!!!!!!!!!!!!!!!!!!!!!
#define H11  0.007548889315f
#define H12  0.0002899088594f
#define H13 -1.290254073f
#define H21  8.625606481e-05f
#define H22 -0.004575319953f
#define H23  1.733411797f
#define H31 -0.0006622785396f
#define H32  0.02125916896f
#define H33  1.0f

/* Pixy2 line-tracking grid (79x52) scaled to calibration frame (316x208) */
#define PIXY_GRID_SCALE    4.0f

/* Track geometry */
#define LANE_WIDTH_M       0.20f          /* distance between the two lane lines (meters) */
#define MAX_LINE_ANGLE_DEG 40.0f          /* reject lines angled more than this from forward */

void pixel_to_ground(float u, float v, float *X, float *Y);
void print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index);

typedef struct {
    float x0_ground, y0_ground, x1_ground, y1_ground; // real-world cm, not pixels
    float angle_deg;
    uint8_t index;
} track_vector_t;

/* Two-boundary lane centering result */
typedef struct {
    float cte;            /* lateral offset of lane center from car (meters, + = right) */
    float heading;        /* average lane heading (radians, + = veering right)           */
    uint8_t lines_found;  /* 0 = no valid lines, 1 = one boundary, 2 = both boundaries  */
} lane_result_t;

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors);

#endif