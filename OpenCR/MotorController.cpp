#include "MotorController.h"

// Constructor: Initializes the MotorController with a Dynamixel2Arduino instance.
MotorController::MotorController(Dynamixel2Arduino& dxl) 
: dxl_(dxl) {
    // Initialize standard_encoders and standard_angles arrays
    for (int i = 0; i <= NUM_MOTORS; ++i) {
        standard_encoders_[i] = 0;
        standard_angles_[i] = 0.0f;
    }
    // Set specific standard angles for motors based on the original code logic
    standard_angles_[MotorID::MOTOR_ID_4] = 90.0f; // Motor 4 was 90 degrees
}

// Initializes a single Dynamixel motor.
void MotorController::initializeMotor(uint8_t motor_id, uint16_t current_limit) {
    dxl_.torqueOff(motor_id);
    // The original code used OP_EXTENDED_POSITION for all motors.
    dxl_.setOperatingMode(motor_id, OP_EXTENDED_POSITION);
    dxl_.writeControlTableItem(ControlTableItem::CURRENT_LIMIT, motor_id, current_limit);
    delay(10); // Short delay as in original code
    dxl_.torqueOn(motor_id);
}

// Sets the standard encoder value for a motor based on its current position.
void MotorController::setStandardEncoder(uint8_t motor_id, int32_t current_position) {
    if (motor_id > 0 && motor_id <= NUM_MOTORS) {
        // Original logic: if current position >= 2048, standard encoder is 4096, else 0.
        standard_encoders_[motor_id] = (current_position >= (ENCODER_RESOLUTION / 2.0f)) ? ENCODER_RESOLUTION : 0;
        DEBUG_SERIAL.print("Motor ");
        DEBUG_SERIAL.print(motor_id);
        DEBUG_SERIAL.print(" (Current Pos: ");
        DEBUG_SERIAL.print(current_position);
        DEBUG_SERIAL.print(") -> Standard updated to: ");
        DEBUG_SERIAL.println(standard_encoders_[motor_id]);
    } else {
        DEBUG_SERIAL.print("Invalid Motor ID for setStandardEncoder: ");
        DEBUG_SERIAL.println(motor_id);
    }
}

// Converts a target angle in degrees to a Dynamixel encoder value.
int32_t MotorController::angleToEncoder(uint8_t motor_id, float target_angle_deg) {
    if (motor_id > 0 && motor_id <= NUM_MOTORS) {
        int32_t std_encoder = standard_encoders_[motor_id];
        float   std_angle   = standard_angles_[motor_id];

        // Calculate the difference in angle and convert to encoder steps
        float delta_angle = target_angle_deg - std_angle;
        int32_t delta_encoder = (int32_t)(delta_angle * ENCODER_PER_DEG);

        // Apply motor direction and add to standard encoder
        return std_encoder + (MOTOR_DIRECTIONS[motor_id] * delta_encoder);
    }
    DEBUG_SERIAL.print("Invalid Motor ID for angleToEncoder: ");
    DEBUG_SERIAL.println(motor_id);
    return 0; // Return 0 for invalid motor ID
}

// Helper function to set control table items for a motor
void MotorController::setMotorControlTableItems(uint8_t motor_id, const MotorControlParams& params) {
    dxl_.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, motor_id, params.p_gain);
    dxl_.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, motor_id, params.d_gain);
    dxl_.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, motor_id, params.acceleration);
    dxl_.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, motor_id, params.velocity);
}

// Moves a single motor to a target angle with specified control parameters.
void MotorController::moveMotor(uint8_t motor_id, float target_angle_deg, const MotorControlParams& params) {
    setMotorControlTableItems(motor_id, params);
    int32_t goal_position_encoder = angleToEncoder(motor_id, target_angle_deg);
    dxl_.setGoalPosition(motor_id, goal_position_encoder);
}

// Moves motors 2 and 3 simultaneously to their target angles using sync write.
void MotorController::syncMoveMotor2And3(
    float target_angle_motor2_deg, float target_angle_motor3_deg,
    const MotorControlParams& params
) {
    // Set control parameters for motor 2
    setMotorControlTableItems(MotorID::MOTOR_ID_2, params);
    // Set control parameters for motor 3 (using the same parameters as motor 2 as in original code)
    setMotorControlTableItems(MotorID::MOTOR_ID_3, params);

    int32_t goal_motor2 = angleToEncoder(MotorID::MOTOR_ID_2, target_angle_motor2_deg);
    int32_t goal_motor3 = angleToEncoder(MotorID::MOTOR_ID_3, target_angle_motor3_deg);

    // Prepare sync write parameters
    ParamForSyncWriteInst_t sw_param;
    sw_param.addr = ADDR_GOAL_POSITION;
    sw_param.length = LEN_GOAL_POSITION;
    sw_param.id_count = 2;

    sw_param.xel[0].id = MotorID::MOTOR_ID_2;
    memcpy(sw_param.xel[0].data, &goal_motor2, sizeof(goal_motor2));
    sw_param.xel[1].id = MotorID::MOTOR_ID_3;
    memcpy(sw_param.xel[1].data, &goal_motor3, sizeof(goal_motor3));

    DEBUG_SERIAL.println("Moving Motor 2 & 3 Simultaneously...");
    dxl_.syncWrite(sw_param); 
}

// Turns off torque for a single motor.
void MotorController::torqueOff(uint8_t motor_id) {
    dxl_.torqueOff(motor_id);
}

// Turns on torque for a single motor.
void MotorController::torqueOn(uint8_t motor_id) {
    dxl_.torqueOn(motor_id);
}

// Pings a motor to check its connection status.
bool MotorController::pingMotor(uint8_t motor_id) {
    if (dxl_.ping(motor_id)) {
        return true;
    } else {
        DEBUG_SERIAL.print("Motor ");
        DEBUG_SERIAL.print(motor_id);
        DEBUG_SERIAL.println(" PING FAIL");
        return false;
    }
}
