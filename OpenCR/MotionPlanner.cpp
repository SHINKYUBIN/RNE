#include "MotionPlanner.h"

MotionPlanner::MotionPlanner(Kinematics& kinematics, MotorController& motor_controller)
    : kinematics_(kinematics), motor_controller_(motor_controller) {
}

void MotionPlanner::executeKinematicsSequence(
    float end_effector_x, float end_effector_y,
    float gripper_dx, float gripper_dy,
    const MotorControlParams& params,
    const GripperState& gripper_state,
    bool apply_delay_after_motor_move
) {
    // 역기구학 계산을 통해 각 관절의 목표 각도(라디안)를 얻습니다.
    JointAngles angles = kinematics_.solveInverseKinematics(
        end_effector_x, end_effector_y, gripper_dx, gripper_dy
    );

    // 만약 계산된 각도가 유효하지 않다면 (도달 불가능한 위치) 함수를 종료합니다.
    if (
        angles.theta1 == 0 && angles.theta2 == 0 && angles.theta3 == 0 &&
        angles.theta4 == 0 && angles.theta5 == 0 && angles.theta6 == 0 &&
        (end_effector_x != 0 || end_effector_y != 0 || gripper_dx != 0 || gripper_dy != 0) // Only return if it's not the initial zero state
    ) {
        DEBUG_SERIAL.println("Kinematics: Target unreachable or invalid.");
        return;
    }

    // 2. 모터 구동 시퀀스: 6 -> 5 -> 4 -> 1 -> (2&3 동시)
    // 이 순서는 로봇 팔의 물리적 구조와 안정성을 고려하여 결정됩니다.
    if (apply_delay_after_motor_move) DEBUG_SERIAL.println("Sequence: 6 -> 5 -> 4 -> 1 -> 2&3");

    // 6번 모터 구동 (그리퍼 회전)
    // `gripper_motor_six_active` 플래그가 `false`일 경우 모터 구동을 건너뜁니다.
    if (gripper_state.motor_six_active == false) {
        motor_controller_.moveMotor(MotorID::MOTOR_ID_6, angles.theta6 * RAD_TO_DEG, params);
        if (apply_delay_after_motor_move) delay(DELAY_MOTOR_MOVE_MS);
    }

    // 5번 모터 구동 (그리퍼 롤)
    // `gripper_motor_five_active` 플래그가 `false`일 경우 모터 구동을 건너뜁니다.
    if (gripper_state.motor_five_active == false) {
        motor_controller_.moveMotor(MotorID::MOTOR_ID_5, angles.theta5 * RAD_TO_DEG, params);
        if (apply_delay_after_motor_move) delay(DELAY_MOTOR_MOVE_MS);
    }

    // 4번 모터 구동 (손목 피치)
    motor_controller_.moveMotor(MotorID::MOTOR_ID_4, angles.theta4 * RAD_TO_DEG, params);
    if (apply_delay_after_motor_move) delay(DELAY_MOTOR_MOVE_MS);

    // 1번 모터 구동 (베이스 회전)
    motor_controller_.moveMotor(MotorID::MOTOR_ID_1, angles.theta1 * RAD_TO_DEG, params);
    if (apply_delay_after_motor_move) delay(DELAY_MOTOR_MOVE_MS);

    // 2번, 3번 모터 동시 구동 (어깨, 팔꿈치)
    // 이 두 모터는 기구학적으로 동시에 움직여야 하는 경우가 많아 동기화하여 제어합니다.
    motor_controller_.syncMoveMotor2And3(angles.theta2 * RAD_TO_DEG, angles.theta2 * RAD_TO_DEG, params);
    
    if (apply_delay_after_motor_move) delay(DELAY_MOTOR_MOVE_MS + 100); // 동시 구동 후 약간 더 긴 대기
}

void MotionPlanner::executeLinearMotion(
    float start_x, float start_y, 
    float end_x, float end_y,
    float gripper_dx, float gripper_dy,
    int num_steps,
    const MotorControlParams& params,
    const GripperState& gripper_state
) {
    DEBUG_SERIAL.println("--- Starting Linear Motion ---");
    for (int i = 1; i <= num_steps; i++) {
        float progress_ratio = (float)i / num_steps;
        
        // 현재 단계의 목표 X, Y 좌표를 선형 보간으로 계산합니다.
        float current_x = start_x + (end_x - start_x) * progress_ratio;
        float current_y = start_y + (end_y - start_y) * progress_ratio;
        
        // 선형 이동 시에는 부드러운 연결을 위해 executeKinematicsSequence 내부의 딜레이를 끕니다.
        executeKinematicsSequence(current_x, current_y, gripper_dx, gripper_dy, params, gripper_state, false);
        
        // 모터가 웨이포인트를 따라갈 수 있도록 짧은 대기 시간 부여
        delay(DELAY_SHORT_WAYPOINT_MS); 
    }
    DEBUG_SERIAL.println("--- Linear Motion Complete ---");
}

void MotionPlanner::calculateHookPosition(
    float center_x, float center_y,
    float gripper_dx, float gripper_dy,
    float& hook_x, float& hook_y
) {
    float gripper_direction_norm = normalizeVector(gripper_dx, gripper_dy);
    hook_x = center_x + LINEAR_PUSH_DISTANCE_MM * gripper_dx / gripper_direction_norm;
    hook_y = center_y + LINEAR_PUSH_DISTANCE_MM * gripper_dy / gripper_direction_norm;
}

void MotionPlanner::calculateReturnPosition(
    float target_x, float target_y,
    float gripper_dx, float gripper_dy,
    float& return_x, float& return_y
) {
    float return_norm = normalizeVector(gripper_dx, gripper_dy);
    return_x = target_x - LINEAR_PUSH_DISTANCE_MM * gripper_dx / return_norm;
    return_y = target_y - LINEAR_PUSH_DISTANCE_MM * gripper_dy / return_norm;
}

void MotionPlanner::moveToInitialPosition(uint8_t motor_id, const MotorControlParams& params) {
    if(!motor_controller_.pingMotor(motor_id)) return;

    float target_start_angle_deg = 0.0f;
    switch(motor_id) {
        case MotorID::MOTOR_ID_2: target_start_angle_deg = 160.0f; break;
        case MotorID::MOTOR_ID_3: target_start_angle_deg = 160.0f; break;
        case MotorID::MOTOR_ID_4: target_start_angle_deg = 44.0f;  break;
        case MotorID::MOTOR_ID_5: target_start_angle_deg = 0.0f;   break;
        case MotorID::MOTOR_ID_6: target_start_angle_deg = 0.0f;   break; 
    }
    motor_controller_.moveMotor(motor_id, target_start_angle_deg, params);
}

float MotionPlanner::normalizeVector(float dx, float dy) {
    return sqrt(dx * dx + dy * dy);
}