#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "Configuration.h"

// Forward declaration for Dynamixel2Arduino
class Dynamixel2Arduino;

// Struct to hold motor parameters for control
struct MotorControlParams {
    uint32_t velocity;      // Profile velocity
    uint32_t acceleration;  // Profile acceleration
    uint16_t p_gain;        // Position P Gain
    uint16_t d_gain;        // Position D Gain
};

class MotorController {
public:
    // Constructor: Initializes the MotorController with a Dynamixel2Arduino instance.
    // Parameters:
    //   dxl: Reference to the Dynamixel2Arduino object for motor communication.
    MotorController(Dynamixel2Arduino& dxl);

    // Initializes a single Dynamixel motor.
    // Sets operating mode, current limit, and torques on the motor.
    // Parameters:
    //   motor_id: The ID of the motor to initialize.
    //   current_limit: The current limit to set for the motor.
    void initializeMotor(uint8_t motor_id, uint16_t current_limit = DEFAULT_CURRENT_LIMIT);

    // Sets the standard encoder value for a motor based on its current position.
    // This is used for relative angle calculations.
    // Parameters:
    //   motor_id: The ID of the motor.
    //   current_position: The present position read from the motor.
    void setStandardEncoder(uint8_t motor_id, int32_t current_position);

    // Converts a target angle in degrees to a Dynamixel encoder value.
    // Takes into account the motor's standard encoder, standard angle, and direction.
    // Parameters:
    //   motor_id: The ID of the motor.
    //   target_angle_deg: The desired angle in degrees.
    // Returns: The corresponding Dynamixel encoder value.
    int32_t angleToEncoder(uint8_t motor_id, float target_angle_deg);

    // Moves a single motor to a target angle with specified control parameters.
    // Parameters:
    //   motor_id: The ID of the motor to move.
    //   target_angle_deg: The target angle in degrees.
    //   params: Struct containing velocity, acceleration, P_gain, D_gain.
    void moveMotor(uint8_t motor_id, float target_angle_deg, const MotorControlParams& params);

    // Moves motors 2 and 3 simultaneously to their target angles using sync write.
    // This is specific to the robot's design where these two motors move in tandem.
    // Parameters:
    //   target_angle_motor2_deg: Target angle for motor 2 in degrees.
    //   target_angle_motor3_deg: Target angle for motor 3 in degrees.
    //   params: Struct containing velocity, acceleration, P_gain, D_gain for both motors.
    void syncMoveMotor2And3(
        float target_angle_motor2_deg, float target_angle_motor3_deg,
        const MotorControlParams& params
    );

    // Turns off torque for a single motor.
    void torqueOff(uint8_t motor_id);

    // Turns on torque for a single motor.
    void torqueOn(uint8_t motor_id);

    // Pings a motor to check its connection status.
    bool pingMotor(uint8_t motor_id);

private:
    Dynamixel2Arduino& dxl_;
    // Standard encoder values for each motor, used as a reference point.
    // Index 0 is a dummy value for 1-based motor ID indexing.
    int32_t standard_encoders_[NUM_MOTORS + 1];
    // Standard angles for each motor, corresponding to the standard encoder values.
    // Index 0 is a dummy value for 1-based motor ID indexing.
    float standard_angles_[NUM_MOTORS + 1];

    // Sets the control table items for a given motor based on the provided parameters.
    // This is a helper function to avoid code duplication in `moveMotor` and `syncMoveMotor2And3`.
    void setMotorControlTableItems(uint8_t motor_id, const MotorControlParams& params);
};

#endif // MOTOR_CONTROLLER_H