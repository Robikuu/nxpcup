#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ---- Pixy2 Frame & Track Geometry ----
#define PIXY_FRAME_WIDTH             79.0f
#define PIXY_CENTER_X                39.5f
#define PIXY_TRACK_HALF_WIDTH        25.0f

// ---- Top Camera Chassis Exclusion Zone (Ignores Front Bumper, Wheels, Servo & Tie-Rods) ----
// Covers entire front axle, wheels at full steering deflection, servo horn, and chassis plate.
#define TOP_CAM_EXCLUDE_Y_MIN        38U       // Pixy2 frame Y >= 38 (~290-400 in PixyMon, covers axle & wheels)
#define TOP_CAM_EXCLUDE_X_MIN        18U       // Pixy2 frame X >= 18 (~145 in PixyMon, covers left wheel lock)
#define TOP_CAM_EXCLUDE_X_MAX        62U       // Pixy2 frame X <= 62 (~505 in PixyMon, covers right wheel lock)

// ---- Physical Steering Limits & Trim Offset ----
#define STEERING_LIMIT_RIGHT         65.0f     // Physical servo right mechanical limit
#define STEERING_LIMIT_LEFT         -65.0f     // Physical servo left mechanical limit
#define STEERING_OFFSET              12.5f     // Straight-line trim offset (trimmed to eliminate rightward lane drift)

// Relative command limits (steer_cmd relative to center trim)
#define STEER_CMD_MAX_RIGHT          (STEERING_LIMIT_RIGHT - STEERING_OFFSET)  // +52.5f
#define STEER_CMD_MAX_LEFT           (STEERING_LIMIT_LEFT - STEERING_OFFSET)   // -77.5f

// ---- Wheel Speeds & Asymmetric Differential Drag (Fast Race Profile) ----
#define DEFAULT_DRIVE_SCALE          1.00f     // 100% full output power
#define SPEED_STRAIGHT               100.0f    // Straight line sprint speed (%)
#define SPEED_OUTSIDE_CORNER         58.0f     // Calibrated DRV8871 apex outside wheel drive (%)
#define SPEED_INSIDE_MIN             38.0f     // Calibrated inside wheel pivot anchor duty (%)

// ---- Differential Drag Scaling ----
#define DIFFERENTIAL_DEADZONE_DEG    6.0f      // Steer angle deadzone below which zero differential drag applies (bites at 6°)
#define STEER_FULL_BRAKE_DEG         45.0f     // Steer angle (deg) for 100% pivot drag ramp

// ---- Straight-Line Active Threshold Pre-Braking ----
#define PREBRAKE_PULSE_TICKS         3U        // 3 ticks @ 50Hz = 60ms active low-side brake pulse
#define PREBRAKE_MIN_SPEED           80.0f     // Only brake if sprinting at or above 80%
#define PREBRAKE_MAX_HEADING_DEG     10.0f     // Straight line alignment guard
#define PREBRAKE_MAX_STEER_DEG       12.0f     // Wheels centered alignment guard
#define PREBRAKE_LOOKAHEAD_THRESH    0.35f     // Front camera turn urgency threshold
#define PREBRAKE_CONFIRM_TICKS       2U        // Consecutive ticks required to confirm turn

// ---- Bilateral Anti-Spin Traction Guard (per 20ms tick) ----
#define DECEL_RAMP_STEP              28.0f     // Rapid pre-braking ramp (~50ms from 100% to 78% at 12.6V)
#define ACCEL_RAMP_STEP              35.0f     // Crisp corner exit acceleration (~60ms to 100%, eliminates straightaway delay)

// ---- Direction-Reversal Transition Ramp (S-curves / chicanes) ----
#define REVERSAL_RELEASE_STEP        20.0f     // Unloads previous dragging inside wheel without breaking rear traction
#define REVERSAL_DECEL_STEP          18.0f     // Rapidly locks new inside wheel into pivot drag
#define YAW_CONFLICT_DEADBAND        3.0f      // Steer angle deadband to trigger yaw conflict logic

// ---- Forward Camera Lookahead Horizon & Root Anchor Geometry ----
#define FORWARD_CAM_HORIZON_Y_MIN    13U       // Discards upper 25% of forward camera view (walls/lights)
#define FORWARD_CAM_ROOT_ANCHOR_Y_MIN 30U      // Vector base must anchor in foreground bumper zone (Y >= 30)
#define FORWARD_CAM_SAMPLE_HORIZON_Y 20.0f     // Far preview horizon (~1.5m ahead) for speed governor pre-braking
#define FORWARD_CAM_SAMPLE_NEAR_Y    38.0f     // Near steering horizon (~0.45m ahead) for fallback steering & feedforward
#define FORWARD_CAM_LEFT_MAX_X       34.0f     // Left root vector base must lie in X <= 34.0
#define FORWARD_CAM_RIGHT_MIN_X      45.0f     // Right root vector base must lie in X >= 45.0
#define LOOKAHEAD_DEADBAND_PX        3.5f      // Neutral straightaway curvature margin (+/- 3.5px)
#define LOOKAHEAD_FULL_URGENCY_PX    12.0f     // Lateral offset/curvature for 100% lookahead urgency

// ---- Dynamic PID Gain Scheduling (Rationalized Decoupled Architecture) ----
#define KP_STRAIGHT                  0.75f     // Damped straight-line centering (no twitch)
#define KP_CORNER                    2.60f     // Crisp cornering authority (decisive bite into 90 deg turns)
#define KD_STRAIGHT                  0.200f    // Lateral velocity damping (calibrated to damp straightaway weave)
#define KD_CORNER                    0.180f    // Lateral velocity damping
#define DEFAULT_KI                   0.0f
#define DEFAULT_ALPHA                0.25f     // Low-pass filter coefficient on derivative term
#define MAX_STEER_RATE_PER_SEC       650.0f    // Max steering slew rate (deg/s) -> 13.0 deg/tick

// ---- Error Weights & Decoupled Feedforward ----
#define DEFAULT_K_CTE                0.55f     // Calibrated CTE weight (smooth straight line centering)
#define DEFAULT_K_HEADING            1.80f     // Direct base camera heading feedforward into curves (strong turn-in bite)
#define DEFAULT_K_LOOKAHEAD          0.0f      // Lookahead steering injection disabled (eliminates premature turn-in)
#define HEADING_DEADBAND_DEG         5.0f      // Heading deadband below which base feedforward is 0.0 (prevents straight jitter)

// ---- Control Loop ----
#define DEFAULT_DT                   0.02f     // 20ms tick = 50Hz
#define DEFAULT_SHADOW_MODE          1U        // 1 = shadow mode (top steers, front logs for calibration)
#define DEFAULT_MOTOR_ENABLE         1U        // 1 = drive motors active, 0 = drive motors disabled for push calibration

// ---- Dual-Camera Steering Blend & Fallback ----
#define FRONT_KP                     0.70f     // Calibrated front camera proportional steering gain (1:1 parity with top)
#define FRONT_K_OFFSET               0.25f     // Calibrated front camera horizon lateral offset gain
#define BLEND_FALL_SLEW_MAX          0.50f     // 2 ticks (40ms) fast transfer to front camera on top dropout
#define BLEND_RISE_SLEW_MAX          0.08f     // ~12 ticks (240ms) smooth return to top camera on reacquisition
#define BLEND_FALLBACK_SPEED_CAP     65.0f     // Max speed when front camera dominates (alpha < 0.30)

// ---- State Machine & Macro States ----
#define MACRO_STATE_NORMAL           0
#define MACRO_STATE_INTERSECTION     1
#define MACRO_STATE_LINE_LOST        2

// ---- Continuous Curvature Governor (Noise Deadbands & Saturation) ----
#define GOVERNOR_HEADING_DEADBAND_DEG 5.0f     // Noise floor below which base heading curvature is 0.0 (stops phantom braking)
#define GOVERNOR_HEADING_MAX_DEG      30.0f    // Heading angle corresponding to 100% corner intensity
#define INTERSECTION_TIMEOUT_TICKS    18U      // 18 ticks (360ms) fixed crossing timeout
#define LINE_LOST_DEBOUNCE_TICKS      3U       // 3 ticks (60ms) optical dropout debounce before entering LINE_LOST
#define LINE_LOST_BRAKE_TICKS         18U      // 18 ticks (360ms) coast window before active braking
#define LINE_LOST_STOP_TICKS          50U      // 50 ticks (1000ms) before full emergency stop
#define LINE_REACQUIRE_TICKS          3U       // 3 consecutive valid frames required to return to NORMAL

// ---- Geometric Boundary Validation ----
#define MIN_LANE_WIDTH_PX            32.0f     // Minimum lane width (rejects center-dash at 25px, preserves tight curves at 34px)
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
    float k_lookahead;
    float dt;
    float accel_ramp;
    float decel_ramp;
    float drive_scale;           // Output plant gain scale
    uint8_t shadow_mode_enable; // 1 = shadow mode (steer on top, log front), 0 = active blend
    uint8_t motor_enable;        // 1 = drive motors active, 0 = drive motors disabled for push-calibration
    uint8_t state;
} robot_params_t;

extern robot_params_t g_params;

void params_init(void);

#endif // CONFIG_H
