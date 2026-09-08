#ifndef VECTORS_H
#define VECTORS_H

#include <math.h>
#include <stdint.h>
/* old homogeneous transformation matrix 
#define H11  0.24541846f
#define H12  0.01463179f
#define H13 -83.82410126f
#define H21  0.00291846f
#define H22 -0.16027502f
#define H23  99.86513131f
#define H31 -0.00021669f
#define H32  0.00594291f
#define H33  1.00000000f
*/

/* new homogeneous transformation matrix, calibrated 2024-06-05 */
#define H11  1.97267759f
#define H12  0.11677262f
#define H13 -83.82452547f
#define H21  0.02345854f
#define H22 -1.27910538f
#define H23  99.86462818f
#define H31 -0.00174181f
#define H32  0.04742887f
#define H33  1.00000000f

/* Pixy2 line-tracking grid (79x52) scaled to calibration frame (316x208) */
#define PIXY_GRID_SCALE    4.0f

/* Track geometry */
#define LANE_WIDTH_M       0.50f          /* distance between the two lane lines (meters) */
#define MAX_LINE_ANGLE_DEG 40.0f          /* reject lines angled more than this from forward */

#define TRACK_HALF_WIDTH_CM 25.75f
#define DEADZONE_CM 1.0f
#define SINGLE_VECTOR_ROTATION_SIGN -1
#define SINGLE_VECTOR_MAX_AGE_TICKS 5000
#define SINGLE_VECTOR_MAX_JUMP_CM 1000


/* A single track-boundary line, already in ground-plane cm.
 * (X0,Y0) and (X1,Y1) are NOT guaranteed to be "near"/"far" in that
 * order — Pixy doesn't guarantee endpoint order, so anything that
 * consumes this must sort by Y itself (see interp_x_at_y below). */
typedef struct {
    float   X0, Y0;
    float   X1, Y1;
    uint8_t valid;
} ground_vector_t;

typedef struct {
    float    cte_est;      // reference centerline position, cm
    uint8_t  initialized;
    uint32_t age_ticks;    // ticks since last 2-line update
} tracked_center_t;

typedef enum { LEFT, RIGHT, UNCERTAIN } side_t;

typedef enum {
    SVEST_OK = 0,
    SVEST_NO_REF,
    SVEST_STALE,
    SVEST_BAD_SIDE,
    SVEST_DEGENERATE,
    SVEST_IMPLAUSIBLE
} svest_reason_t;

/* The centerline the PID should track: a 2-point vector (extensible
 * to more points / a spline later when curvature gets added), plus
 * derived scalars for convenience.
 *
 * heading is in RADIANS (raw atan2f output) — if your PID/control
 * code expects degrees, convert explicitly at the call site. Do not
 * assume; this is exactly the kind of unit mismatch that fails
 * silently and produces a plausible-looking but wrong response. */
typedef struct {
    float   X0, Y0;   // nearest point of the centerline (within the
                       // Y-range BOTH boundary lines actually covered
                       // — not extrapolated out to Y=0 / the car)
    float   X1, Y1;   // farthest jointly-observed point
    float   cte;       // == X0. Lateral offset at the near point.
    float   heading;   // radians, atan2f(X1-X0, Y1-Y0). 0 = straight ahead, + = right.
    uint8_t valid;
} center_line_t;

/* Two-boundary lane centering result */
typedef struct {
    float cte;            /* lateral offset of lane center from car (meters, + = right) */
    float heading;        /* average lane heading (radians, + = veering right)           */
    uint8_t lines_found;  /* 0 = no valid lines, 1 = one boundary, 2 = both boundaries  */
} lane_result_t;



void print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index);
uint8_t pixel_to_ground(float u, float v, float *X, float *Y);
void update_tracked_center(tracked_center_t *ref, float cte_est);
void age_tracked_center(tracked_center_t *ref);
side_t classify_side(float x, const tracked_center_t *ref);
uint8_t select_boundary_vectors(const uint16_t *vectors, size_t num_vectors,
                                 const tracked_center_t *ref,
                                 ground_vector_t *left, ground_vector_t *right,
                                 side_t *single_side);
center_line_t compute_center_line(const ground_vector_t *left, const ground_vector_t *right);
uint8_t estimate_single_vector_cte(const ground_vector_t *v, side_t side,
                                    const tracked_center_t *ref, float *cte_out);
svest_reason_t estimate_single_vector_cte_debug(const ground_vector_t *v, side_t side,
                                                 const tracked_center_t *ref, float *cte_out);
void print_frame_debug(const uint16_t *vectors, size_t num_vectors);

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors);

#endif
