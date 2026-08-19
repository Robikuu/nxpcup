#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include "wifi_config.h"
#include "vectors.h"

#define MIN_VALID_LANE_WIDTH_PX 28.0f
#define MAX_VALID_LANE_WIDTH_PX 52.0f
#define DEFAULT_HALF_LANE_PX    19.0f

static float s_tracked_half_lane = DEFAULT_HALF_LANE_PX;
static float s_last_left_x       = PIXY_CENTER_X - DEFAULT_HALF_LANE_PX;
static float s_last_right_x      = PIXY_CENTER_X + DEFAULT_HALF_LANE_PX;

void reset_lane_tracker(void)
{
    s_tracked_half_lane = DEFAULT_HALF_LANE_PX;
    s_last_left_x       = PIXY_CENTER_X - DEFAULT_HALF_LANE_PX;
    s_last_right_x      = PIXY_CENTER_X + DEFAULT_HALF_LANE_PX;
}

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors)
{
    lane_result_t result = {0.0f, 0.0f, 0};
    if (num_vectors == 0) return result;

    typedef struct {
        float x_at_look;
        float slope;
    } cand_t;

    #define MAX_LOCAL_CANDS 16
    cand_t cands[MAX_LOCAL_CANDS];
    size_t cand_count = 0;

    for (size_t i = 0; i < num_vectors && cand_count < MAX_LOCAL_CANDS; i++) {
        float x0 = (float)vectors[6*i + 0];
        float y0 = (float)vectors[6*i + 1];
        float x1 = (float)vectors[6*i + 2];
        float y1 = (float)vectors[6*i + 3];

        /* Ensure (x0, y0) is near (larger Y) and (x1, y1) is far (smaller Y) */
        if (y0 < y1) {
            float tx = x0; float ty = y0;
            x0 = x1; y0 = y1;
            x1 = tx; y1 = ty;
        }

        float dx = x1 - x0;
        float dy = y0 - y1;
        if (dy < MIN_VECTOR_DY) continue;

        float len_sq = dx * dx + dy * dy;
        if (len_sq < (MIN_VECTOR_LEN * MIN_VECTOR_LEN)) continue;

        float abs_slope = fabsf(dx / dy);
        if (abs_slope > INTERSECTION_MAX_SLOPE) continue;

        float slope = dx / dy;
        float eval_y = (float)LOOKAHEAD_Y;
        if (eval_y < y1) eval_y = y1;
        if (eval_y > y0) eval_y = y0;
        float x_at_look = x0 + slope * (y0 - eval_y);

        cands[cand_count].x_at_look = x_at_look;
        cands[cand_count].slope     = slope;
        cand_count++;
    }

    if (cand_count == 0) return result;

    /* Sort candidates left-to-right */
    for (size_t i = 0; i < cand_count - 1; i++) {
        for (size_t j = i + 1; j < cand_count; j++) {
            if (cands[j].x_at_look < cands[i].x_at_look) {
                cand_t tmp = cands[i];
                cands[i] = cands[j];
                cands[j] = tmp;
            }
        }
    }

    /* Merge adjacent piecewise segments on the same boundary (< 16px gap) */
    cand_t merged[MAX_LOCAL_CANDS];
    size_t merged_count = 0;
    for (size_t i = 0; i < cand_count; i++) {
        if (merged_count > 0 && (cands[i].x_at_look - merged[merged_count - 1].x_at_look) < 16.0f) {
            merged[merged_count - 1].x_at_look = 0.5f * (merged[merged_count - 1].x_at_look + cands[i].x_at_look);
            merged[merged_count - 1].slope     = 0.5f * (merged[merged_count - 1].slope + cands[i].slope);
        } else {
            merged[merged_count++] = cands[i];
        }
    }

    if (merged_count >= 2) {
        float best_diff = 1e6f;
        int best_i = -1, best_j = -1;
        float target_w = s_tracked_half_lane * 2.0f;

        for (size_t i = 0; i < merged_count - 1; i++) {
            for (size_t j = i + 1; j < merged_count; j++) {
                float w = merged[j].x_at_look - merged[i].x_at_look;
                if (w >= MIN_VALID_LANE_WIDTH_PX && w <= MAX_VALID_LANE_WIDTH_PX) {
                    float score = fabsf(w - target_w);
                    if (score <= 14.0f && score < best_diff) {
                        best_diff = score;
                        best_i = (int)i;
                        best_j = (int)j;
                    }
                }
            }
        }

        if (best_i >= 0 && best_j >= 0) {
            float left_x   = merged[best_i].x_at_look;
            float right_x  = merged[best_j].x_at_look;
            float center_x = 0.5f * (left_x + right_x);
            float width    = right_x - left_x;

            s_tracked_half_lane = 0.85f * s_tracked_half_lane + 0.15f * (0.5f * width);
            s_last_left_x       = left_x;
            s_last_right_x      = right_x;

            result.cte         = center_x - PIXY_CENTER_X;
            result.heading     = 0.5f * (merged[best_i].slope + merged[best_j].slope);
            result.lines_found = 2;
            return result;
        }
    }

    /* Single line fallback: select candidate closest to tracked boundary memory */
    cand_t *c = &merged[0];
    float min_dist = 1e6f;
    for (size_t i = 0; i < merged_count; i++) {
        float dl = fabsf(merged[i].x_at_look - s_last_left_x);
        float dr = fabsf(merged[i].x_at_look - s_last_right_x);
        float d  = (dl < dr) ? dl : dr;
        if (d < min_dist) {
            min_dist = d;
            c = &merged[i];
        }
    }

    float d_to_left  = fabsf(c->x_at_look - s_last_left_x);
    float d_to_right = fabsf(c->x_at_look - s_last_right_x);
    float center_x;

    if (d_to_left < d_to_right) {
        center_x       = c->x_at_look + s_tracked_half_lane;
        s_last_left_x  = c->x_at_look;
        s_last_right_x = c->x_at_look + (2.0f * s_tracked_half_lane);
    } else {
        center_x       = c->x_at_look - s_tracked_half_lane;
        s_last_right_x = c->x_at_look;
        s_last_left_x  = c->x_at_look - (2.0f * s_tracked_half_lane);
    }

    result.cte         = center_x - PIXY_CENTER_X;
    result.heading     = c->slope;
    result.lines_found = 1;
    return result;
}

size_t filter_intersection_vectors(uint16_t *vectors, size_t num_vectors)
{
    if (num_vectors == 0) return 0;
    size_t kept = 0;

    for (size_t i = 0; i < num_vectors; i++) {
        float x0 = (float)vectors[6*i + 0];
        float y0 = (float)vectors[6*i + 1];
        float x1 = (float)vectors[6*i + 2];
        float y1 = (float)vectors[6*i + 3];

        if (y0 < y1) {
            float tx = x0; float ty = y0;
            x0 = x1; y0 = y1;
            x1 = tx; y1 = ty;
        }

        float dx = x1 - x0;
        float dy = y0 - y1;
        if (dy < MIN_VECTOR_DY) continue;

        float len_sq = dx * dx + dy * dy;
        if (len_sq < (MIN_VECTOR_LEN * MIN_VECTOR_LEN)) continue;

        float abs_slope = fabsf(dx / dy);
        if (abs_slope > INTERSECTION_MAX_SLOPE) continue;

        if (kept != i) {
            for (int j = 0; j < 6; j++) {
                vectors[6*kept + j] = vectors[6*i + j];
            }
        }
        kept++;
    }
    return kept;
}

void print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index)
{
    int dx = (int)x1 - (int)x0;
    int dy = (int)y1 - (int)y0;

    float length = sqrtf((float)(dx * dx + dy * dy));

    float angle_deg = 0.0f;
    if (dy != 0) {
        angle_deg = atan2f((float)dx, (float)(-dy)) * 180.0f / 3.141592653589793f;
    } else {
        angle_deg = (dx >= 0) ? 90.0f : -90.0f;
    }

    int len_int = (int)length;
    int len_frac = (int)((length - len_int) * 100.0f);
    if (len_frac < 0) len_frac = -len_frac;

    int ang_int = (int)angle_deg;
    int ang_frac = (int)((angle_deg - (float)ang_int) * 100.0f);
    if (ang_frac < 0) ang_frac = -ang_frac;

    esp_printf("Vector [%u] details:\r\n", (unsigned)index);
    esp_printf("  Start: (%u, %u) -> End: (%u, %u)\r\n", (unsigned)x0, (unsigned)y0, (unsigned)x1, (unsigned)y1);
    esp_printf("  dx: %d, dy: %d\r\n", dx, dy);
    esp_printf("  Length: %d.%02d px\r\n", len_int, len_frac);
    esp_printf("  Angle: %d.%02d deg\r\n", ang_int, ang_frac);
}
