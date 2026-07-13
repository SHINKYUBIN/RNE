#include "Kinematics.h"

// Constructor implementation
Kinematics::Kinematics(
    float link1_length, float link2_length, float link3_length,
    float link4_length, float gripper_offset_radius
) : 
    L1_(link1_length), L2_(link2_length), L3_(link3_length),
    L4_(link4_length), R_gripper_offset_(gripper_offset_radius)
{}

JointAngles Kinematics::solveInverseKinematics(
    float target_x, float target_y,
    float target_dx, float target_dy
) {
    JointAngles angles = {0, 0, 0, 0, 0, 0}; // Initialize with zeros

    // 1. Calculate theta1 (Base rotation)
    angles.theta1 = atan2(target_y, target_x);

    // Calculate distance from Joint1 to the wrist joint (considering gripper offset)
    float dist_to_wrist = sqrt(
        pow(target_x - R_gripper_offset_ * cos(angles.theta1), 2) +
        pow(target_y - R_gripper_offset_ * sin(angles.theta1), 2)
    );

    float dr = dist_to_wrist - L4_; // Horizontal projection to the shoulder-elbow plane
    float dz = L1_;                  // Vertical distance to the shoulder-elbow plane

    // Total distance from shoulder to wrist (D)
    float D = sqrt(pow(dr, 2) + pow(dz, 2));

    // Check if the target is reachable (triangle inequality)
    if (D > (L2_ + L3_) || D < fabs(L2_ - L3_)) {
        // If unreachable, return initialized (zero) angles
        return angles;
    }

    // 2. Calculate theta6 (Gripper rotation relative to base)
    angles.theta6 = constrain(atan2(target_dy, target_dx) - angles.theta1, THETA6_MIN_RAD, THETA6_MAX_RAD);

    // 3. Calculate theta2 (Shoulder joint angle)
    // Using Law of Cosines to find internal angles of the shoulder-elbow-wrist triangle
    float cos_val_alpha = (pow(L2_, 2) + pow(D, 2) - pow(L3_, 2)) / (2.0f * L2_ * D);
    angles.theta2 = THETA2_OFFSET_RAD + atan2(dr, dz) + acos(constrain(cos_val_alpha, -1.0f, 1.0f));

    // 4. Calculate theta4 (Wrist joint pitch angle)
    float cos_val_beta = (pow(L2_, 2) + pow(L3_, 2) - pow(D, 2)) / (2.0f * L2_ * L3_);
    angles.theta4 = acos(constrain(cos_val_beta, -1.0f, 1.0f)) - THETA2_OFFSET_RAD;
    
    // 5. Calculate theta5 (Wrist joint roll angle) (derived from theta2 and theta4)
    angles.theta5 = constrain(M_PI - angles.theta2 - angles.theta4, 0.0f, M_PI/2.0f);

    return angles;
}

void Kinematics::convertPixelToMM(float pixel_x, float pixel_y, float &real_x, float &real_y) {
    // Coefficients derived from camera calibration.
    // These values transform pixel coordinates into the robot's millimeter workspace.
    constexpr float COEFF_DENOM_X_PX = -0.000215923f; // Coefficient for xp in denominator
    constexpr float COEFF_DENOM_Y_PX = 0.000446514f;  // Coefficient for yp in denominator
    constexpr float COEFF_DENOM_CONST = 1.0f;        // Constant in denominator

    constexpr float COEFF_XR_X_PX = 0.002227866f;  // Coefficient for xp in xr numerator
    constexpr float COEFF_XR_Y_PX = -0.183991437f; // Coefficient for yp in xr numerator
    constexpr float COEFF_XR_CONST = 70.2117635f; // Constant in xr numerator
    constexpr float SCALE_XR = 10.0f;               // Scaling factor for xr

    constexpr float COEFF_YR_X_PX = -0.210088564f; // Coefficient for xp in yr numerator
    constexpr float COEFF_YR_Y_PX = 0.009742372f;  // Coefficient for yp in yr numerator
    constexpr float COEFF_YR_CONST = 130.614924f; // Constant in yr numerator
    constexpr float SCALE_YR = 10.0f;               // Scaling factor for yr

    // Calculate the common denominator for both x and y transformations
    float denominator = COEFF_DENOM_X_PX * pixel_x + COEFF_DENOM_Y_PX * pixel_y + COEFF_DENOM_CONST;

    // Calculate real-world x coordinate in mm
    real_x = SCALE_XR * (
        COEFF_XR_X_PX * pixel_x + COEFF_XR_Y_PX * pixel_y + COEFF_XR_CONST
    ) / denominator;

    // Calculate real-world y coordinate in mm
    real_y = SCALE_YR * (
        COEFF_YR_X_PX * pixel_x + COEFF_YR_Y_PX * pixel_y + COEFF_YR_CONST
    ) / denominator;
}
