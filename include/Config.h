#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ---- Pixy2 Frame & Track Geometry ----
#define PIXY_FRAME_WIDTH             79.0f
#define PIXY_CENTER_X                39.5f
#define PIXY_TRACK_HALF_WIDTH        25.0f

// ---- Top Camera Chassis Exclusion Zone (Ignores Front Bumper, Wheels, Servo & Tie-Rods) ----
#define TOP_CAM_EXCLUDE_Y_MIN        38U       // Pixy2 frame Y >= 38
#define TOP_CAM_EXCLUDE_X_MIN        18U       // Pixy2 frame X >= 18
#define TOP_CAM_EXCLUDE_X_MAX        62U       // Pixy2 frame X <= 62

// ---- Physical Steering Limits & Trim Offset ----
#define STEERING_LIMIT_RIGHT         77.0f     // Physical servo right mechanical limit
#define STEERING_LIMIT_LEFT         -67.0f     // Physical servo left mechanical limit
#define STEERING_OFFSET              10.00f     // Straight-line trim offset

// Relative command limits (steer_cmd relative to center trim)
#define STEER_CMD_MAX_RIGHT          (STEERING_LIMIT_RIGHT - STEERING_OFFSET)
#define STEER_CMD_MAX_LEFT           (STEERING_LIMIT_LEFT - STEERING_OFFSET)

// ---- Wheel Speeds & Asymmetric Differential Drag ----
#define DEFAULT_DRIVE_SCALE          1.00f     // 100% full output power
#define SPEED_STRAIGHT               100.0f     // Straight line sprint speed (%)
#define SPEED_OUTSIDE_CORNER         62.0f     // Calibrated apex outside wheel drive (%)
#define SPEED_INSIDE_MIN             32.0f     // Inside wheel pivot anchor duty (%)

// ---- Differential Drag Scaling ----
#define DIFFERENTIAL_DEADZONE_DEG    8.0f      // Steer angle deadzone below which zero differential drag applies
#define STEER_FULL_BRAKE_DEG         26.0f     // Steer angle (deg) for 100% pivot drag ramp

// ---- Dual-Camera Horizon & Lane Constants ----
#define FRONT_CORNER_ANGLE_THRESH_DEG 30.0f    // Horizon road angle threshold for sharp corner detection
#define TRACK_HALF_WIDTH_PX          25.0f     // Nominal lane half-width (pixels) in top camera view
#define FRONT_FEEDFORWARD_GAIN       0.60f     // Pre-steer feedforward gain when corner detected

// ---- Severity-Scaled Corner Speed (speed = spd_s - severity * (spd_s - spd_out)) ----
#define FRONT_SEVERITY_FULL_DEG      40.0f     // Front angle (deg) where front severity reaches 1.0 (ramp starts at fa_th)
#define CORNER_SEVERITY_HOLD_TICKS   6U        // Ticks the severity peak is held after it drops (120ms bridge + flicker guard)

// ---- Bilateral Anti-Spin Traction Guard (per 20ms tick) ----
#define DECEL_RAMP_STEP              30.0f     // Rapid pre-braking & pivot lock ramp (~2 ticks from 100% to 40%)
#define ACCEL_RAMP_STEP              6.5f      // Smooth traction-preserving corner exit acceleration (~160ms)

// ---- Error Weights & Heading Deadband ----
#define DEFAULT_K_CTE                0.55f     // Calibrated CTE weight (smooth straight line centering)
#define DEFAULT_K_HEADING            0.90f     // Direct base camera heading damping
#define HEADING_DEADBAND_DEG         2.0f      // Heading deadband on straights to prevent pixel jitter

// ---- Control Loop & Failsafe ----
#define DEFAULT_MOTOR_ENABLE         1U        // 1 = drive motors active, 0 = drive motors disabled for push calibration
#define OFF_TRACK_FAILSAFE_TICKS     125U      // 125 ticks (2.5 seconds @ 50Hz) off-track emergency stop

// ---- Geometric Boundary Validation ----
#define MIN_LANE_WIDTH_PX            32.0f     // Minimum lane width to reject duplicate split line segments
#define MAX_LANE_WIDTH_PX            68.0f     // Maximum lane width

// ---- Continuous Asymmetric Corner Blend Governor ----
#define DEFAULT_H_LO                 12.0f     // Heading deadband floor (deg): below this, hold = 0.0
#define DEFAULT_H_HI                 35.0f     // Heading saturation ceiling (deg): above this, hold = 1.0
#define DEFAULT_BLEND_ATTACK         0.35f     // Fast attack rate (~100ms / 5 ticks to 88% corner engagement)
#define DEFAULT_BLEND_RELEASE        0.06f     // Smooth release rate (~25-30 ticks gradual throttle recovery)
#define DEFAULT_FRONT_ANGLE_THRESH   23.0f     // Front camera angle threshold (deg) where corner blend begins
#define DEFAULT_BLEND_ENABLE         0U        // 0 = legacy discrete mode default, 1 = continuous blend active
#define DEFAULT_K_CTE_CORNER         DEFAULT_K_CTE
#define DEFAULT_K_HEADING_CORNER     DEFAULT_K_HEADING

// ---- Corner-Exit / Off-Center Recovery (all tick counts are 50 Hz loop ticks; 4 ticks = 1 telemetry line) ----
#define STEER_SYMMETRIC_CLAMP        1U        // 1 = clamp steer_cmd to +/-min(right,left) throw around trim so CW == CCW authority
#define FRONT_STALE_DECAY_NET        0.65f     // Net per-tick filtered_front_angle factor when top cam is straight+centered and front cam sees nothing (0.65^4=0.18/line)
#define STALE_FRONT_MAX_HEADING_DEG  15.0f     // Top-camera |heading| below which front-angle fast decay / corner release is allowed
#define STALE_FRONT_MAX_CTE_PX       12.0f     // Top-camera |cte| below which front-angle fast decay is allowed
#define CORNER_RELEASE_STRAIGHT_TICKS 6U       // Consecutive straight 2-line ticks (120 ms) before stale corner state is cleared
#define CORNER_RELEASE_LANE_MIN_PX   35.0f     // Valid lane width window for corner release
#define CORNER_RELEASE_LANE_MAX_PX   68.0f
#define OFFCENTER_HOLD_CTE_PX        10.0f     // L:0 recovery hold + bypass of +/-3 deg intersection steer clamp above this |cte|
#define OFFCENTER_HOLD_MAX_TICKS     24U       // 24 ticks = 480 ms of gentle decay during blind gap
#define OFFCENTER_HOLD_CTE_DECAY     0.975f    // per tick (0.975^4 = 0.90 per telemetry line)
#define OFFCENTER_HOLD_HEAD_DECAY    0.96f     // per tick (0.96^4  = 0.85 per telemetry line)
#define IX_BLOCK_CTE_PX              14.0f     // Do not latch an intersection when |filtered_cte| exceeds this ...
#define IX_BLOCK_HEADING_DEG         20.0f     // ... or |filtered_heading| exceeds this (boundary crossed diagonally)

// ---- Robot States ----
#define STATE_IDLE                   0
#define STATE_RUNNING                1

// ---- Runtime-Tunable Parameter Struct (18 Active Fields) ----
typedef struct {
    float speed_straight;
    float speed_outside_corner;
    float speed_inside_min;
    float steering_offset;
    float k_cte;
    float k_heading;
    float k_cte_corner;
    float k_heading_corner;
    float h_lo;
    float h_hi;
    float blend_attack;
    float blend_release;
    float front_angle_thresh;
    float accel_ramp;
    float decel_ramp;
    float drive_scale;           // Output plant gain scale (0.10 to 1.00)
    uint8_t blend_enable;        // 1 = continuous blend active, 0 = legacy discrete mode
    uint8_t motor_enable;        // 1 = drive motors active, 0 = drive motors disabled for push-calibration
    uint8_t state;
} robot_params_t;

extern robot_params_t g_params;

void params_init(void);

#endif // CONFIG_H
