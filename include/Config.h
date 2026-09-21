#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ---- Pixy2 Frame & Track Geometry ----
#define PIXY_FRAME_WIDTH             79.0f
#define PIXY_FRAME_HEIGHT            52.0f
#define PIXY_CENTER_X                39.5f
#define PIXY_TRACK_HALF_WIDTH        25.0f

// ---- Physical Steering Limits & Trim Offset ----
#define STEERING_LIMIT_RIGHT         87.0f     // Physical servo right mechanical limit
#define STEERING_LIMIT_LEFT         -70.0f     // Physical servo left mechanical limit
#define STEERING_OFFSET              14.0f     // Straight-line trim offset (+17.0 deg)

// Relative command limits (steer_cmd relative to center trim)
#define STEER_CMD_MAX_RIGHT          (STEERING_LIMIT_RIGHT - STEERING_OFFSET)  // +70.0f
#define STEER_CMD_MAX_LEFT           (STEERING_LIMIT_LEFT - STEERING_OFFSET)   // -87.0f

// ---- Wheel Speeds & Asymmetric Differential Drag (Fast Sprint Profile) ----
#define SPEED_STRAIGHT               100.0f    // Straight line sprint speed (%) (full 100% battery voltage push)
#define SPEED_OUTSIDE_CORNER         80.0f     // Fast apex outside wheel drive (%) (carries momentum through turns)
#define SPEED_INSIDE_MIN             45.0f     // Inside wheel pivot anchor duty (%) (effective pivot drag without bogging)
#define SPEED_CORNER_AVG_MIN         ((SPEED_OUTSIDE_CORNER + SPEED_INSIDE_MIN) * 0.5f) // 62.5%

#define SPEED_RIGHT                  SPEED_STRAIGHT
#define SPEED_LEFT                   SPEED_STRAIGHT

// ---- Bilateral Anti-Spin Traction Guard (per 20ms tick) ----
#define DECEL_RAMP_STEP              16.0f     // Rapid pre-braking ramp (~75ms from 95% to 48%)
#define ACCEL_RAMP_STEP              20.0f     // Punchy corner exit acceleration ramp (~50ms back to 100%)

// ---- Direction-Reversal Transition Ramp (S-curves / chicanes) ----
#define REVERSAL_RELEASE_STEP        35.0f     // Rapidly unloads previous dragging inside wheel
#define REVERSAL_DECEL_STEP          18.0f     // Rapidly locks new inside wheel into pivot drag
#define YAW_CONFLICT_DEADBAND        3.0f      // Steer angle deadband to trigger yaw conflict logic
#define DIFFERENTIAL_DEADZONE_DEG    6.0f      // Steer angle deadzone below which zero differential drag applies

// ---- Corner Urgency & Predictive Lookahead Gating ----
#define CTE_URGENCY_PX               24.0f     // |filtered_cte| (px) at which corner gains engage (prevents straight hunting)
#define CORNER_URGENCY_STREAK        1         // Immediate engagement on rising corner urgency
#define STEER_FULL_BRAKE_DEG         30.0f     // Steer angle (deg) for 100% pivot drag (decisive bite at apex)
#define HEADING_FULL_BRAKE_DEG       22.0f     // Heading angle (deg) for full corner intensity scaling
#define APEX_CUT_RATIO               0.20f     // Fraction of half-lane-width to shift target inside corners (0.0=center, 0.30=aggressive)

// ---- Dynamic PID Gain Scheduling (Anti-Weave Damped) ----
#define KP_CORNER                    2.20f     // Crisp cornering authority without open-loop gain oscillation
#define KP_STRAIGHT                  0.60f     // Calm straight-line tracking authority to eliminate weaving
#define KD_CORNER                    0.120f    // Counter-steer damping in corners
#define KD_STRAIGHT                  0.220f    // Strong derivative damping to lock high-speed straightaway tracking
#define DEFAULT_KI                   0.0f
#define DEFAULT_ALPHA                0.3f

// ---- Error Weights ----
#define DEFAULT_K_CTE                0.50f     // Moderate weight to prevent local CTE from canceling corner turn-in
#define DEFAULT_K_HEADING            0.80f     // Decisive preview heading to initiate corner entry on time
 
// ---- Control Loop ----
#define DEFAULT_DT                   0.02f     // 20ms tick = 50Hz

// ---- State Machine Thresholds ----
#define CORNER_HEADING_THRESHOLD_DEG 18.0f     // Fast corner detection threshold (prevents straightaway oscillation chatter)
#define STATE_ENTER_STREAK           1         // Immediate transition into DRIVE_TURNING (20ms response)
#define STATE_EXIT_STREAK            5         // Ticks before returning to DRIVE_FORWARD
#define LINE_LOST_LIMIT              5
#define LINE_LOST_BRAKE_TICKS        3         // Debounce frames before cutting power and hard braking

// ---- Geometric Boundary Validation ----
#define MIN_LANE_WIDTH_PX            40.0f     // Minimum lane width (rejects adjacent parallel lines / chicane waves)
#define MAX_LANE_WIDTH_PX            68.0f     // Maximum lane width
#define CONNECT_DIST_PX              15.0f     // Endpoint distance threshold to reject split curve fragments

// ---- Robot States ----
#define STATE_IDLE                   0
#define STATE_RUNNING                1

// ---- Runtime-Tunable Parameter Struct ----
typedef struct {
    float kp_straight;
    float kp_corner;
    float ki;
    float kd_straight;
    float kd_corner;
    float speed_straight;
    float speed_outside_corner;
    float speed_inside_min;
    float alpha;
    float steering_offset;
    float k_cte;
    float k_heading;
    float dt;
    float accel_ramp;
    float decel_ramp;
    uint8_t state;
} robot_params_t;

extern robot_params_t g_params;

void params_init(void);

#endif // CONFIG_H
