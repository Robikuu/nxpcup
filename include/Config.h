#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// Steering angle offset
#define STEERING_OFFSET      17.0f

// Wheel speeds
#define SPEED_STRAIGHT        80.0f
#define SPEED_CORNER_MIN      33.0f
#define SPEED_RIGHT           SPEED_STRAIGHT
#define SPEED_LEFT            SPEED_STRAIGHT

#define SPEED_AT_FULL_DUTY_MMPS  2000.0f
#define WHEELBASE_MM        150.0
#define TRACK_WIDTH_MM      500.0
#define MAX_SERVO_ANGLE_DEG  60
#define MAX_LATERAL_ACCEL     3.0
#define BASE_SPEED           100.0


// Differential parameters
#define DIFFERENTIAL_GAIN     0.20f

// Traction Control (Acceleration Ramp)
#define ACCEL_RAMP_STEP       1.0f

#define KP_CORNER             3.0f
#define KP_STRAIGHT           7.0f
#define KD_CORNER             0.5f
#define KD_STRAIGHT           1.3f
#define DEFAULT_KI            0.0f     
#define DEFAULT_ALPHA         0.3f     
#define DEFAULT_K_CTE         1.0f     
#define DEFAULT_K_HEADING     0.30f    

#define HEADING_FULL_BRAKE_DEG  15.0f
#define DEFAULT_DT            0.02f    

// ---- State machine thresholds ----
#define CORNER_HEADING_THRESHOLD_DEG   20.0f
#define STATE_ENTER_STREAK             3    
#define STATE_EXIT_STREAK              5    
#define LINE_LOST_LIMIT                5

// ---- Boundary validation ----
#define MIN_LANE_WIDTH_PX              20.0f
#define CONNECT_DIST_PX                15.0f

// ---- Robot states ----
#define STATE_IDLE     0
#define STATE_RUNNING  1

// ---- Runtime-tunable parameter struct ----
// Default values come from the #define constants above.
// Modified at runtime via Wi-Fi commands received through the ESP32 bridge.
typedef struct {
    float kp_straight;       
    float kp_corner;         
    float ki;                
    float kd_straight;       
    float kd_corner;         
    float speed_straight;    
    float speed_corner;      
    float alpha;                  
    float max_speed_left;         
    float max_speed_right;        
    float steering_offset;        
    float k_cte;                  
    float k_heading;              
    float dt;                     
    float accel_ramp;          
    float differential_gain;
    uint8_t state;            
} robot_params_t;

extern robot_params_t g_params;

void params_init(void);

#endif
