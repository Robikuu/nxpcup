#ifndef VECTORS_H
#define VECTORS_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    float   cte;
    float   heading_deg;
    uint8_t found;
} steering_result_t;

typedef struct {
    float   trajectory_angle_deg;  /* Near path angle relative to vehicle axis (-45 to +45 deg, sampled at near horizon Y=38) */
    float   trajectory_offset_px;  /* Near path lateral displacement from center line (sampled at near horizon Y=38) */
    float   lookahead_x;           /* Projected centerline X coordinate at far horizon Y=20 */
    float   curvature;             /* Path curvature indicator */
    float   lookahead_urgency;     /* 0.0 to 1.0 urgency for pre-braking (evaluated at far horizon Y=20) */
    uint8_t left_root_found;       /* 1 if left foreground root vector detected */
    uint8_t right_root_found;      /* 1 if right foreground root vector detected */
    uint8_t has_transverse_tape;   /* 1 if horizontal crossing tape detected */
    uint8_t found;                 /* Total root boundary vectors found (0, 1, or 2) */
} trajectory_result_t;

void reset_steering_tracking(void);
void vector_filtering(uint16_t *vectors, size_t *num_vectors);
steering_result_t compute_steering_error(const uint16_t *vectors, size_t num_vectors);
trajectory_result_t compute_lookahead_trajectory(const uint16_t *vectors, size_t num_vectors);
trajectory_result_t compute_lookahead_curvature(const uint16_t *vectors, size_t num_vectors);

#endif // VECTORS_H