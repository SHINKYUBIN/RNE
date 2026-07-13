#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <math.h>
#include "Configuration.h"

// Forward declarations for Dynamixel2Arduino dependency
class Dynamixel2Arduino;

// Define a struct to hold calculated joint angles in radians
struct JointAngles {
    float theta1; // Base rotation
    float theta2; // Shoulder joint
    float theta3; // Elbow joint (implied by theta5 and theta4)
    float theta4; // Wrist joint (pitch)
    float theta5; // Wrist joint (roll)
    float theta6; // Gripper rotation
};

class Kinematics {
public:
    // Constructor: Initializes the kinematics solver with robot link lengths
    Kinematics(
        float link1_length, float link2_length, float link3_length,
        float link4_length, float gripper_offset_radius
    );

    // Solves inverse kinematics to find joint angles for a given end-effector pose.
    // Parameters:
    //   target_x, target_y: Target (x, y) coordinates of the end-effector in mm.
    //   target_dx, target_dy: Target direction vector (dx, dy) of the gripper.
    // Returns: JointAngles struct containing the calculated angles in radians.
    //          Returns default (zero) angles if the target is unreachable.
    JointAngles solveInverseKinematics(
        float target_x, float target_y, 
        float target_dx, float target_dy
    );

    // Converts pixel coordinates from a camera system to real-world millimeter coordinates.
    // This function is based on a pre-calibrated transformation matrix.
    // Parameters:
    //   pixel_x, pixel_y: Input pixel coordinates.
    //   real_x, real_y: Output real-world coordinates in mm.
    void convertPixelToMM(float pixel_x, float pixel_y, float &real_x, float &real_y);

private:
    // Robot link lengths and gripper offset
    float L1_, L2_, L3_, L4_, R_gripper_offset_;
};

#endif // KINEMATICS_H