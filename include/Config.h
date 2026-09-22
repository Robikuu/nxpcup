#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ---- Pixy2 Frame & Track Geometry ----
#define PIXY_FRAME_WIDTH             79.0f
#define PIXY_FRAME_HEIGHT            52.0f
#define PIXY_CENTER_X                39.5f
#define PIXY_TRACK_HALF_WIDTH        25.0f

// ---- Top Camera Chassis Exclusion Zone (Ignores Front Bumper & Wheels) ----
// PixyMon viewport (640x400): X in [200, 450], Y in [360, 400]
// Scaled to Pixy2 line frame (79x52): X in [24, 56], Y in [46, 51]
#define TOP_CAM_EXCLUDE_Y_MIN        46U       // Pixy2 frame Y >= 46 (~354-400 in PixyMon)
#define TOP_CAM_EXCLUDE_X_MIN        24U       // Pixy2 frame X >= 24 (~194 in PixyMon)
#define TOP_CAM_EXCLUDE_X_MAX        56U       // Pixy2 frame X <= 56 (~453 in PixyMon)

// ---- Physical Steering Limits & Trim Offset ----
#define STEERING_LIMIT_RIGHT         65.0f     // Physical servo right mechanical limit
#define STEERING_LIMIT_LEFT         -65.0f     // Physical servo left mechanical limit
#define STEERING_OFFSET              12.5f     // Straight-line trim offset (trimmed to eliminate rightward lane drift)

// Relative command limits (steer_cmd relative to center trim)
#define STEER_CMD_MAX_RIGHT          (STEERING_LIMIT_RIGHT - STEERING_OFFSET)  // +52.5f
#define STEER_CMD_MAX_LEFT           (STEERING_LIMIT_LEFT - STEERING_OFFSET)   // -77.5f

// ---- Wheel Speeds & Asymmetric Differential Drag (Fast Sprint Profile) ----
#define SPEED_STRAIGHT               100.0f    // Straight line sprint speed (%) (full 100% battery voltage push)
#define SPEED_OUTSIDE_CORNER         78.0f     // Fast apex outside wheel drive (%) (carries momentum within tire friction circle)
#define SPEED_INSIDE_MIN             58.0f     // Inside wheel pivot anchor duty (%) (effective pivot drag without bogging)
#define SPEED_CORNER_AVG_MIN         ((SPEED_OUTSIDE_CORNER + SPEED_INSIDE_MIN) * 0.5f) // 68.0%

#define SPEED_RIGHT                  SPEED_STRAIGHT
#define SPEED_LEFT                   SPEED_STRAIGHT

// ---- Bilateral Anti-Spin Traction Guard (per 20ms tick) ----
#define DECEL_RAMP_STEP              22.0f     // Rapid pre-braking ramp (~50ms from 100% to 78%)
#define ACCEL_RAMP_STEP              16.0f     // Progressive corner exit acceleration (~120ms to 100%, maintains front tire grip)

// ---- Battery Voltage Compensation ----
#define CALIBRATION_BATTERY_VOLTAGE  11.9f     // Nominal tuning reference voltage (V) where car was calibrated
#define DEFAULT_BATTERY_VOLTAGE      12.6f     // Current charged battery voltage (V)

// ---- Direction-Reversal Transition Ramp (S-curves / chicanes) ----
#define REVERSAL_RELEASE_STEP        20.0f     // Unloads previous dragging inside wheel without breaking rear traction
#define REVERSAL_DECEL_STEP          18.0f     // Rapidly locks new inside wheel into pivot drag
#define YAW_CONFLICT_DEADBAND        3.0f      // Steer angle deadband to trigger yaw conflict logic
#define DIFFERENTIAL_DEADZONE_DEG    3.5f      // Steer angle deadzone below which zero differential drag applies (crisp turn-in bite)

// ---- Corner Urgency & Predictive Lookahead Gating ----
#define CTE_URGENCY_PX               24.0f     // |filtered_cte| (px) at which corner gains engage (prevents straight hunting)
#define CORNER_URGENCY_STREAK        1         // Immediate engagement on rising corner urgency
#define STEER_FULL_BRAKE_DEG         30.0f     // Steer angle (deg) for 100% pivot drag (decisive bite at apex)
#define HEADING_FULL_BRAKE_DEG       22.0f     // Heading angle (deg) for full corner intensity scaling
#define APEX_CUT_RATIO               0.20f     // Fraction of half-lane-width to shift target inside corners (0.0=center, 0.30=aggressive)

// ---- Forward Camera Horizon & Track Floor Gating ----
// Pixy2 frame height is 52px (Y=0 is top horizon/sky, Y=51 is bottom ground).
// Discard upper quarter (Y < 13) to eliminate background ceiling/walls/room clutter.
// Require candidate vectors to anchor on the track floor (Y_near >= 24).
#define FORWARD_CAM_HORIZON_Y_MIN    13U       // Discards upper 25% of forward camera view
#define FORWARD_CAM_FLOOR_Y_MIN      24U       // Lookahead vector base must touch track floor

// ---- Dynamic PID Gain Scheduling (Decoupled Feedforward/Feedback) ----
#define KP_CORNER                    1.40f     // Decisive centering in corners
#define KP_STRAIGHT                  0.80f     // Firm centering on straights
#define KD_CORNER                    0.180f    // Active derivative damping on lateral velocity d(CTE)/dt
#define KD_STRAIGHT                  0.150f    // High-speed straightaway damping
#define DEFAULT_KI                   0.0f
#define DEFAULT_ALPHA                0.25f     // Low-pass filter coefficient on derivative term
#define MAX_STEER_RATE_PER_SEC       600.0f    // Max steering slew rate (deg/s) prevents chassis snapping

// ---- Error Weights ----
#define DEFAULT_K_CTE                0.60f     // Calibrated CTE weight
#define DEFAULT_K_HEADING            0.85f     // Direct heading feedforward into curves
 
// ---- Control Loop ----
#define DEFAULT_DT                   0.02f     // 20ms tick = 50Hz

// ---- State Machine Thresholds ----
#define CORNER_HEADING_THRESHOLD_DEG 15.0f     // Calibrated corner entry heading threshold (prevents straightaway twitch)
#define STATE_ENTER_STREAK           1         // Immediate transition into DRIVE_TURNING (20ms response)
#define STATE_EXIT_STREAK            5         // Ticks before returning to DRIVE_FORWARD
#define LINE_LOST_LIMIT              18U       // ~360ms coast window for traversing intersection gaps without braking
#define LINE_LOST_BRAKE_TICKS        3         // Debounce frames before cutting power and hard braking

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
    float dt;
    float accel_ramp;
    float decel_ramp;
    float battery_voltage;
    float vbat_scale;
    uint8_t state;
} robot_params_t;

extern robot_params_t g_params;

void params_init(void);

#endif // CONFIG_H
