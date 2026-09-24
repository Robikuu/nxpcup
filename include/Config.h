#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ---- Pixy2 Frame & Track Geometry ----
#define PIXY_FRAME_WIDTH             79.0f
#define PIXY_FRAME_HEIGHT            52.0f
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

// ---- Wheel Speeds & Asymmetric Differential Drag (12.6V Full-Power Race Profile) ----
#define SPEED_STRAIGHT               100.0f    // Straight line sprint speed (%) (full 12.6V battery voltage push)
#define SPEED_OUTSIDE_CORNER         58.0f     // Calibrated apex outside wheel drive (%) (prevents corner understeer push)
#define SPEED_INSIDE_MIN             48.0f     // Inside wheel pivot anchor duty (%) (gentler pivot, 9.7% min yaw torque verified safe)
#define SPEED_CORNER_AVG_MIN         ((SPEED_OUTSIDE_CORNER + SPEED_INSIDE_MIN) * 0.5f) // 53.0%

#define SPEED_RIGHT                  SPEED_STRAIGHT
#define SPEED_LEFT                   SPEED_STRAIGHT

// ---- Bilateral Anti-Spin Traction Guard (per 20ms tick) ----
#define DECEL_RAMP_STEP              25.0f     // Rapid pre-braking ramp (~60ms from 100% to 58% at 12.6V)
#define ACCEL_RAMP_STEP              18.0f     // Crisp corner exit acceleration (~100ms to 100%, maintains front tire grip)

// ---- Direction-Reversal Transition Ramp (S-curves / chicanes) ----
#define REVERSAL_RELEASE_STEP        20.0f     // Unloads previous dragging inside wheel without breaking rear traction
#define REVERSAL_DECEL_STEP          18.0f     // Rapidly locks new inside wheel into pivot drag
#define YAW_CONFLICT_DEADBAND        3.0f      // Steer angle deadband to trigger yaw conflict logic
#define DIFFERENTIAL_DEADZONE_DEG    3.5f      // Steer angle deadzone below which zero differential drag applies (crisp turn-in bite)

// ---- Corner Urgency & Predictive Lookahead Gating ----
#define CTE_URGENCY_PX               24.0f     // |filtered_cte| (px) at which corner gains engage (prevents straight hunting)
#define CORNER_URGENCY_STREAK        1         // Immediate engagement on rising corner urgency
#define STEER_FULL_BRAKE_DEG         45.0f     // Steer angle (deg) for 100% pivot drag (wider ramp, smoother chicane transitions)
#define HEADING_FULL_BRAKE_DEG       22.0f     // Heading angle (deg) for full corner intensity scaling
#define APEX_CUT_RATIO               0.20f     // Fraction of half-lane-width to shift target inside corners (0.0=center, 0.30=aggressive)

// ---- Forward Camera Lookahead Horizon & Root Anchor Geometry ----
#define FORWARD_CAM_HORIZON_Y_MIN    13U       // Discards upper 25% of forward camera view (walls/lights)
#define FORWARD_CAM_ROOT_ANCHOR_Y_MIN 30U      // Vector base must anchor in foreground bumper zone (Y >= 30)
#define FORWARD_CAM_SAMPLE_HORIZON_Y 20.0f     // Lookahead horizon sample line (~1.5m ahead)
#define FORWARD_CAM_LEFT_MAX_X       34.0f     // Left root vector base must lie in X <= 34.0
#define FORWARD_CAM_RIGHT_MIN_X      45.0f     // Right root vector base must lie in X >= 45.0
#define STRAIGHT_SANITY_HEADING_MAX_DEG 4.0f   // Max heading angle (deg) to confirm vehicle is parallel to track
#define STRAIGHT_SANITY_CTE_MAX_PX      10.0f  // Max CTE (px) to confirm vehicle is safely within straight lane bounds
#define LOOKAHEAD_DEADBAND_PX        3.5f      // Neutral straightaway curvature margin (+/- 3.5px)
#define LOOKAHEAD_FULL_URGENCY_PX    12.0f     // Lateral offset/curvature for 100% lookahead urgency
#define LOOKAHEAD_HYSTERESIS_ENTER_PX 7.5f     // Lookahead offset threshold to trigger DRIVE_TURNING
#define LOOKAHEAD_HYSTERESIS_EXIT_PX  4.5f     // Lookahead offset threshold to return to DRIVE_FORWARD
#define LOOKAHEAD_DROPOUT_HOLD_TICKS 3U        // 3 ticks (60ms) hold-last-valid window for optical flicker

// ---- Dynamic PID Gain Scheduling (Rationalized Decoupled Architecture) ----
#define KP_STRAIGHT                  0.75f     // Damped straight-line centering (no twitch)
#define KP_CORNER                    2.60f     // Crisp cornering authority (decisive bite into 90 deg turns)
#define KD_STRAIGHT                  0.180f    // Lateral velocity damping
#define KD_CORNER                    0.180f    // Lateral velocity damping
#define DEFAULT_KI                   0.0f
#define DEFAULT_ALPHA                0.25f     // Low-pass filter coefficient on derivative term
#define MAX_STEER_RATE_PER_SEC       650.0f    // Max steering slew rate (deg/s) -> 13.0 deg/tick

// ---- Error Weights & Decoupled Feedforward ----
#define DEFAULT_K_CTE                0.60f     // Calibrated CTE weight (strong centering authority)
#define DEFAULT_K_HEADING            1.70f     // Direct base camera heading feedforward into curves (strong turn-in bite)
#define DEFAULT_K_LOOKAHEAD          0.35f     // Predictive turn-in feedforward from front camera (bites 40-60ms early)
#define HEADING_DEADBAND_DEG         2.0f      // Heading deadband below which base feedforward is 0.0 (prevents straight jitter)

// ---- Control Loop ----
#define DEFAULT_DT                   0.02f     // 20ms tick = 50Hz
#define DEFAULT_SHADOW_MODE          1U        // 1 = shadow mode (top steers, front logs for calibration)

// ---- State Machine & Macro States ----
#define MACRO_STATE_NORMAL           0
#define MACRO_STATE_INTERSECTION     1
#define MACRO_STATE_LINE_LOST        2

// ---- Continuous Curvature Governor (Noise Deadbands & Saturation) ----
#define GOVERNOR_HEADING_DEADBAND_DEG 2.5f     // Noise floor below which base heading curvature is 0.0
#define GOVERNOR_HEADING_MAX_DEG      25.0f    // Heading angle corresponding to 100% corner intensity
#define GOVERNOR_CURVATURE_DEADBAND_PX 4.0f    // Noise floor below which front lookahead mid_dx is 0.0
#define GOVERNOR_CURVATURE_MAX_PX     14.0f    // Horizon lateral curvature displacement for 100% urgency
#define INTERSECTION_TIMEOUT_TICKS    18U      // 18 ticks (360ms) fixed crossing timeout
#define LINE_LOST_DEBOUNCE_TICKS      3U       // 3 ticks (60ms) optical dropout debounce before entering LINE_LOST
#define LINE_LOST_BRAKE_TICKS         18U      // 18 ticks (360ms) coast window before active braking
#define LINE_LOST_STOP_TICKS          50U      // 50 ticks (1000ms) before full emergency stop
#define LINE_REACQUIRE_TICKS          3U       // 3 consecutive valid frames required to return to NORMAL

// Legacy compatibility definitions
#define CORNER_HEADING_THRESHOLD_DEG 8.0f      // Base camera heading threshold for corner confirmation
#define STATE_ENTER_STREAK           2         // 2 ticks (40ms) sustained curve confirmation
#define STATE_EXIT_STREAK            2         // 2 ticks (40ms) fast return to DRIVE_FORWARD on straight
#define CORNER_APPROACH_ABORT_TICKS  18U       // 18 ticks (360ms) safety timeout to abort corner pre-braking on false alarm
#define LINE_LOST_LIMIT              18U       // ~360ms coast window for traversing intersection gaps without braking

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
    uint8_t shadow_mode_enable; // 1 = shadow mode (steer on top, log front), 0 = active blend
    uint8_t state;
} robot_params_t;

extern robot_params_t g_params;

void params_init(void);

#endif // CONFIG_H
