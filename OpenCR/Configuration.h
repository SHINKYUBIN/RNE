#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

// --- Hardware and Communication Settings ---
#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
#define PI_SERIAL    Serial1

constexpr int32_t DXL_BAUDRATE = 1000000;
constexpr uint32_t RPI_BAUDRATE = 115200;

// --- Robot Kinematics Constants (Measured values) ---
// Joint1 height - shoe height - Joint5 height
constexpr float LINK_LENGTH_1 = 285.0f - 50.0f - 50.0f - 60.0f; 
constexpr float LINK_LENGTH_2 = 540.0f; 
constexpr float LINK_LENGTH_3 = 540.0f; 
constexpr float LINK_LENGTH_4 = 150.0f;  
constexpr float GRIPPER_OFFSET_RADIUS = 60.0f; // Offset from Joint1 to the effective gripping point

// --- Dynamixel Control Table Items (Partial) ---
// Dynamixel control table item addresses and lengths for sync write.
constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
constexpr uint16_t ADDR_GOAL_POSITION = 116;
constexpr uint16_t LEN_GOAL_POSITION = 4;
constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
constexpr uint16_t LEN_PROFILE_VELOCITY = 4;
constexpr uint16_t ADDR_PROFILE_ACCELERATION = 108;
constexpr uint16_t LEN_PROFILE_ACCELERATION = 4;
constexpr uint16_t ADDR_POS_P_GAIN = 84;
constexpr uint16_t LEN_POS_P_GAIN = 2;
constexpr uint16_t ADDR_POS_D_GAIN = 80;
constexpr uint16_t LEN_POS_D_GAIN = 2;
constexpr uint16_t ADDR_CURRENT_LIMIT = 38;
constexpr uint16_t LEN_CURRENT_LIMIT = 2;

// --- Motor IDs ---
enum MotorID {
    MOTOR_ID_1 = 1,
    MOTOR_ID_2 = 2,
    MOTOR_ID_3 = 3,
    MOTOR_ID_4 = 4,
    MOTOR_ID_5 = 5,
    MOTOR_ID_6 = 6
};

const uint8_t ALL_MOTOR_IDS[] = {MOTOR_ID_1, MOTOR_ID_2, MOTOR_ID_3, MOTOR_ID_4, MOTOR_ID_5, MOTOR_ID_6};
const uint8_t NUM_MOTORS = sizeof(ALL_MOTOR_IDS) / sizeof(ALL_MOTOR_IDS[0]);

// --- Motor Directions (to account for physical mounting) ---
// Defines the direction multiplier for each motor to convert calculated angles to encoder values.
// Index 0 is a dummy value for 1-based motor ID indexing.
const int8_t MOTOR_DIRECTIONS[] = {
    0,   // Dummy for 1-based indexing
    1,   // Motor 1
    1,   // Motor 2
    -1,  // Motor 3
    -1,  // Motor 4
    1,   // Motor 5
    -1   // Motor 6
};

// --- Default Motor Control Parameters ---
constexpr uint16_t DEFAULT_P_GAIN = 800;    
constexpr uint16_t DEFAULT_D_GAIN = 1000;    
constexpr uint32_t DEFAULT_VELOCITY = 30;
constexpr uint32_t DEFAULT_ACCELERATION = 10;
constexpr uint16_t DEFAULT_CURRENT_LIMIT = 800;

// --- Angle Conversion ---
constexpr float ENCODER_RESOLUTION = 4096.0f;
constexpr float DEG_PER_ENCODER = 360.0f / ENCODER_RESOLUTION;
constexpr float ENCODER_PER_DEG = ENCODER_RESOLUTION / 360.0f;
constexpr float RAD_TO_DEG = 180.0f / M_PI;
constexpr float DEG_TO_RAD = M_PI / 180.0f;

// --- Kinematics Constraints ---
// Constraints for theta6 (gripper rotation) based on physical limitations.
constexpr float THETA6_MIN_RAD = -1.48f;
constexpr float THETA6_MAX_RAD = 1.48f;
// Offset angle for theta2 calculation (specific to robot design).
constexpr float THETA2_OFFSET_RAD = 13.0f * DEG_TO_RAD;

// --- Timing and Delays ---
constexpr uint32_t DELAY_MOTOR_MOVE_MS = 400; // Delay after individual motor moves
constexpr uint32_t DELAY_SHORT_WAYPOINT_MS = 40; // Short delay for linear motion waypoints
constexpr uint32_t DELAY_KINEMATICS_REACH_MS = 3000; // Delay to reach target after kinematics
constexpr uint32_t DELAY_HOOK_STABILIZE_MS = 1000; // Delay after linear push for stabilization
constexpr uint32_t DELAY_ALIGNMENT_MS = 2000; // Delay after shoe alignment
constexpr uint32_t DELAY_MOTOR_INIT_MS = 1000; // Delay after motor initialization
constexpr uint32_t DELAY_MOTOR1_WAIT_MS = 3000; // Delay before initializing Motor 1
constexpr uint32_t DELAY_SETUP_COMPLETE_MS = 2000; // Delay after setup completion

// --- Linear Motion Parameters ---
constexpr int LINEAR_MOTION_STEPS = 5; // Number of steps for linear interpolation
constexpr float LINEAR_PUSH_DISTANCE_MM = 150.0f; // Distance for shoe hook/push

// --- Shoe Alignment Target (Pixel coordinates from Camera) ---
constexpr float SHOE_ALIGN_TARGET_PX_X = 620.0f; 
constexpr float SHOE_ALIGN_TARGET_PX_Y = 130.0f; 

#endif // CONFIGURATION_H