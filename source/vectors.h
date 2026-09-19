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
    float   trajectory_angle_deg;  /* Ahead path angle relative to vehicle axis (-35 to +35 deg) */
    float   trajectory_offset_px;  /* Ahead path lateral displacement from center line */
    float   curvature;             /* Path curvature indicator */
    float   lookahead_urgency;     /* 0.0 to 1.0 urgency for pre-braking */
    uint8_t found;                 /* Number of valid trajectory vectors detected */
} trajectory_result_t;

void reset_steering_tracking(void);
void vector_filtering(uint16_t *vectors, size_t *num_vectors);
steering_result_t compute_steering_error(const uint16_t *vectors, size_t num_vectors);
trajectory_result_t compute_lookahead_trajectory(const uint16_t *vectors, size_t num_vectors);

#endif // VECTORS_H