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
#define STEERING_LIMIT_RIGHT         65.0f     // Physical servo right mechanical limit
#define STEERING_LIMIT_LEFT         -65.0f     // Physical servo left mechanical limit
#define STEERING_OFFSET              0.00f     // Straight-line trim offset

// Relative command limits (steer_cmd relative to center trim)
#define STEER_CMD_MAX_RIGHT          (STEERING_LIMIT_RIGHT - STEERING_OFFSET)
#define STEER_CMD_MAX_LEFT           (STEERING_LIMIT_LEFT - STEERING_OFFSET)

// ---- Wheel Speeds & Asymmetric Differential Drag ----
#define DEFAULT_DRIVE_SCALE          1.00f     // 100% full output power
#define SPEED_STRAIGHT               100.0f    // Straight line sprint speed (%)
#define SPEED_OUTSIDE_CORNER         55.0f     // Calibrated apex outside wheel drive (%)
#define SPEED_INSIDE_MIN             32.0f     // Inside wheel pivot anchor duty (%)

// ---- Differential Drag Scaling ----
#define DIFFERENTIAL_DEADZONE_DEG    8.0f      // Steer angle deadzone below which zero differential drag applies
#define STEER_FULL_BRAKE_DEG         40.0f     // Steer angle (deg) for 100% pivot drag ramp

// ---- Dual-Camera Horizon & Lane Constants ----
#define FRONT_CORNER_ANGLE_THRESH_DEG 30.0f    // Horizon road angle threshold for sharp corner detection
#define TRACK_HALF_WIDTH_PX          25.0f     // Nominal lane half-width (pixels) in top camera view
#define FRONT_FEEDFORWARD_GAIN       0.45f     // Pre-steer feedforward gain when corner detected

// ---- Severity-Scaled Corner Speed (speed = spd_s - severity * (spd_s - spd_out)) ----
#define FRONT_SEVERITY_FULL_DEG      50.0f     // Front angle (deg) where front severity reaches 1.0 (ramp starts at fa_th)
#define CORNER_SEVERITY_HOLD_TICKS   3U        // Ticks the severity peak is held after it drops (corner wall flicker guard)

// ---- Bilateral Anti-Spin Traction Guard (per 20ms tick) ----
#define DECEL_RAMP_STEP              30.0f     // Rapid pre-braking & pivot lock ramp (~2 ticks from 100% to 40%)
#define ACCEL_RAMP_STEP              6.0f      // Smooth traction-preserving corner exit acceleration (~160ms)

// ---- Error Weights & Heading Deadband ----
#define DEFAULT_K_CTE                0.50f     // Calibrated CTE weight (smooth straight line centering)
#define DEFAULT_K_HEADING            1.10f     // Direct base camera heading damping
#define HEADING_DEADBAND_DEG         2.0f      // Heading deadband on straights to prevent pixel jitter

// ---- Control Loop & Failsafe ----
#define DEFAULT_MOTOR_ENABLE         1U        // 1 = drive motors active, 0 = drive motors disabled for push calibration
#define OFF_TRACK_FAILSAFE_TICKS     125U      // 125 ticks (2.5 seconds @ 50Hz) off-track emergency stop

// ---- Geometric Boundary Validation ----
#define MIN_LANE_WIDTH_PX            32.0f     // Minimum lane width to reject duplicate split line segments
#define MAX_LANE_WIDTH_PX            68.0f     // Maximum lane width

// ---- Continuous Asymmetric Corner Blend Governor ----
#define DEFAULT_H_LO                 18.0f     // Heading deadband floor (deg): below this, hold = 0.0
#define DEFAULT_H_HI                 35.0f     // Heading saturation ceiling (deg): above this, hold = 1.0
#define DEFAULT_BLEND_ATTACK         0.35f     // Fast attack rate (~100ms / 5 ticks to 88% corner engagement)
#define DEFAULT_BLEND_RELEASE        0.06f     // Smooth release rate (~25-30 ticks gradual throttle recovery)
#define DEFAULT_FRONT_ANGLE_THRESH   25.0f     // Front camera angle threshold (deg) where corner blend begins
#define DEFAULT_BLEND_ENABLE         0U        // 0 = legacy discrete mode default, 1 = continuous blend active
#define DEFAULT_K_CTE_CORNER         DEFAULT_K_CTE
#define DEFAULT_K_HEADING_CORNER     DEFAULT_K_HEADING

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
