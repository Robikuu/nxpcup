#ifndef VECTORS_H
#define VECTORS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Top Camera Result: Centered between closest left and right lines */
typedef struct {
    float   cte;             /* Cross-track error in pixels (center_x - 39.0f) */
    float   heading_deg;     /* Road heading angle in degrees relative to vertical */
    uint8_t left_found;          /* 1 if closest left line detected */
    uint8_t right_found;         /* 1 if closest right line detected */
    uint8_t has_transverse_tape; /* 1 if horizontal crossing line detected */
    uint8_t found;               /* Total lines found (0, 1, or 2) */
} steering_result_t;

/* Front Camera Result: Lookahead horizon road angle */
typedef struct {
    float   horizon_angle_deg; /* Length-weighted average forward road angle (-90 to +90 deg) */
    uint8_t is_sharp_corner;   /* 1 if |horizon_angle_deg| > 30.0f */
    uint8_t found;             /* Number of valid horizon vectors detected */
} horizon_result_t;

void reset_steering_tracking(void);
void compute_top_camera_lane(const uint16_t *vectors, size_t num_vectors, steering_result_t *out);
void compute_front_camera_horizon(const uint16_t *vectors, size_t num_vectors, horizon_result_t *out);

#endif // VECTORS_H