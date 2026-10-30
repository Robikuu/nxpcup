#ifndef VECTORS_H
#define VECTORS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Top Camera Result: Centered between closest left and right lines */
typedef struct {
    float   cte;                        /* Cross-track error in pixels (center_x - 39.0f) */
    float   heading_deg;                /* Road heading angle in degrees relative to vertical */
    uint8_t left_found;                 /* 1 if closest left line detected */
    uint8_t right_found;                /* 1 if closest right line detected */
    uint8_t found;                      /* Total lines found (0, 1, or 2) */
    uint8_t has_transverse_tape;        /* 1 if transverse tape detected */
    uint8_t transverse_crosses_center;  /* 1 if transverse tape spans center corridor [32, 46] */
    uint8_t is_intersection_ahead;      /* Reserved / detection flag */
    float   lane_width_px;              /* Projected right-left boundary width (px); 0 unless found == 2 */
} steering_result_t;

/* Front Camera Result: Lookahead horizon road angle */
typedef struct {
    float   horizon_angle_deg;          /* Length-weighted average forward road angle (-90 to +90 deg) */
    uint8_t is_sharp_corner;            /* 1 if |horizon_angle_deg| > 30.0f or corner fail-safe */
    uint8_t found;                      /* Number of valid horizon vectors detected */
    uint8_t valid_longitudinal_count;   /* Number of non-transverse longitudinal lane vectors */
    uint8_t is_intersection_ahead;      /* 1 if confirmed intersection ahead (gap + exit pair) */
    uint8_t has_transverse_tape;        /* 1 if transverse tape detected */
    uint8_t transverse_crosses_center;  /* 1 if transverse tape spans center corridor */
} horizon_result_t;

void reset_steering_tracking(void);
void compute_top_camera_lane(const uint16_t *vectors, size_t num_vectors, steering_result_t *out);
void compute_front_camera_horizon(const uint16_t *vectors, size_t num_vectors, horizon_result_t *out);

#endif // VECTORS_H