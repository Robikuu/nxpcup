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

/* Pixy2 line-tracking grid dimensions (native sensor frame) */
#define PIXY_IMAGE_WIDTH    79
#define PIXY_IMAGE_HEIGHT   52
#define PIXY_CENTER_X       39.0f

/* Nominal half-lane pixel offset for single-line synthesis */
#define HALF_LANE_OFFSET_PX 15.0f

/* Lookahead row for CTE evaluation (0 = top/far, 51 = bottom/near) */
#define LOOKAHEAD_Y         22

/* Intersection detection thresholds: reject crossing vectors steeper than ~37 deg */
#define INTERSECTION_MAX_SLOPE    0.75f

void pixel_to_ground(float u, float v, float *X, float *Y);
void print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index);

typedef struct {
    float x0_ground, y0_ground, x1_ground, y1_ground; // real-world cm, not pixels
    float angle_deg;
    uint8_t index;
} track_vector_t;

/* Two-boundary lane centering result */
typedef struct {
    float   cte;         /* lateral pixel error at lookahead row             */
    float   heading;     /* normalized slope: lateral px per forward px      */
    uint8_t lines_found; /* 0 = lost, 1 = single line, 2 = two lines        */
} lane_result_t;

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors);
size_t filter_intersection_vectors(uint16_t *vectors, size_t num_vectors);
void reset_lane_tracker(void);

#endif