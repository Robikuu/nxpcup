#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "Config.h"
#include "vectors.h"

int main(void) {
    uint16_t sketch_vectors[16] = {
        18, 48, 28, 20,
        62, 48, 52, 20,
        40, 45, 40, 32,
        12, 28,  5, 14
    };
    trajectory_result_t t1 = compute_lookahead_trajectory(sketch_vectors, 4);
    trajectory_result_t t2 = compute_lookahead_curvature(sketch_vectors, 4);
    printf("sketch_vectors:\n");
    printf("  traj:      found=%d, left=%d, right=%d, lookahead_x=%.2f, urgency=%.2f, traj_angle=%.2f\n",
           t1.found, t1.left_root_found, t1.right_root_found, t1.lookahead_x, t1.lookahead_urgency, t1.trajectory_angle_deg);
    printf("  curvature: found=%d, left=%d, right=%d, lookahead_x=%.2f, urgency=%.2f, traj_angle=%.2f\n",
           t2.found, t2.left_root_found, t2.right_root_found, t2.lookahead_x, t2.lookahead_urgency, t2.trajectory_angle_deg);

    uint16_t transverse_vectors[4] = { 15, 25, 65, 24 };
    trajectory_result_t w1 = compute_lookahead_trajectory(transverse_vectors, 1);
    trajectory_result_t w2 = compute_lookahead_curvature(transverse_vectors, 1);
    printf("transverse_vectors:\n");
    printf("  traj:      found=%d, urgency=%.2f, lookahead_x=%.2f, offset=%.2f\n",
           w1.found, w1.lookahead_urgency, w1.lookahead_x, w1.trajectory_offset_px);
    printf("  curvature: found=%d, urgency=%.2f, lookahead_x=%.2f, offset=%.2f\n",
           w2.found, w2.lookahead_urgency, w2.lookahead_x, w2.trajectory_offset_px);
    return 0;
}
