#include "RobotSequence.h"
#include <Dynamixel2Arduino.h>

extern Dynamixel2Arduino dxl; // OpenCR.ino에서 선언된 전역 객체 참조

RobotSequence::RobotSequence(
    MotionPlanner& motion_planner,
    Kinematics& kinematics,
    MotorController& motor_controller,
    CommunicationManager& comm_manager
) : motion_planner_(motion_planner), 
    kinematics_(kinematics), 
    motor_controller_(motor_controller),
    comm_manager_(comm_manager) {
}

void RobotSequence::initialize() {
    // Dynamixel 통신 초기화
    dxl.begin(DXL_BAUDRATE);
    dxl.setPortProtocolVersion(2.0);
    delay(DELAY_MOTOR_INIT_MS); // Dynamixel 초기화 대기
    comm_manager_.debugPrintln("Dynamixel Init Complete");

    // 표준 인코더 설정
    setStandardEncoders();
    
    // 모터 초기화 및 시작 자세 설정
    initializeMotors();
    
    // Motor1 별도 초기화
    initializeMotor1();
    
    comm_manager_.debugPrintln("Setup Complete.");
}

void RobotSequence::pickShoe(const ShoeTarget& target) {
    current_target_ = target;
    
    // 픽셀 좌표를 실제 좌표로 변환
    kinematics_.convertPixelToMM(target.pixel_center_x, target.pixel_center_y, 
                                 current_target_.real_center_x, current_target_.real_center_y);
    kinematics_.convertPixelToMM(target.pixel_source_x, target.pixel_source_y, 
                                 current_target_.real_source_x, current_target_.real_source_y);

    current_target_.gripper_dx = current_target_.real_source_x - current_target_.real_center_x;
    current_target_.gripper_dy = current_target_.real_source_y - current_target_.real_center_y;
    
    comm_manager_.debugPrint("Target Found from Pi: ");
    comm_manager_.debugPrintln(String(current_target_.identifier));
    
    MotorControlParams motion_params = {10, 10, DEFAULT_P_GAIN, 0};

    // 1. 첫 번째 기구학 실행 (목표 신발 위치로 이동)
    comm_manager_.debugPrintln("Executing Kinematics (Moving to shoe)...");
    gripper_state_.motor_five_active = false; // 5번 모터 비활성화
    gripper_state_.motor_six_active = false;  // 6번 모터 비활성화
    motion_planner_.executeKinematicsSequence(
        current_target_.real_center_x, current_target_.real_center_y, 
        current_target_.gripper_dx, current_target_.gripper_dy, 
        motion_params, gripper_state_
    );
    delay(DELAY_KINEMATICS_REACH_MS);

    // 2. 신발 훅(Hook) 로직 - 선형 보간 운동 적용
    comm_manager_.debugPrintln("Performing Linear Push...");
    float hook_target_x, hook_target_y;
    motion_planner_.calculateHookPosition(
        current_target_.real_center_x, current_target_.real_center_y,
        current_target_.gripper_dx, current_target_.gripper_dy,
        hook_target_x, hook_target_y
    );
    
    gripper_state_.motor_six_active = true; // 6번 모터 활성화
    MotorControlParams linear_params = {50, 20, DEFAULT_P_GAIN, 0};
    motion_planner_.executeLinearMotion(
        current_target_.real_center_x, current_target_.real_center_y, 
        hook_target_x, hook_target_y, 
        current_target_.gripper_dx, current_target_.gripper_dy, 
        LINEAR_MOTION_STEPS, linear_params, gripper_state_
    );
    delay(DELAY_HOOK_STABILIZE_MS);

    // 훅 동작 (모터 5번 개별 제어)
    comm_manager_.debugPrintln("Moving Motor 5 (Hooking)...");
    MotorControlParams hook_motor5_params = {10, 5, DEFAULT_P_GAIN, 0};
    motor_controller_.moveMotor(MotorID::MOTOR_ID_5, 60.0f, hook_motor5_params);
    delay(DELAY_ALIGNMENT_MS);
}

void RobotSequence::alignShoe() {
    // 3. 신발 정렬 로직 (최종 목적지로 이동)
    gripper_state_.motor_five_active = true; // 5번 모터 활성화
    gripper_state_.motor_six_active = true;  // 6번 모터 활성화
    
    float final_target_mm_x, final_target_mm_y;
    kinematics_.convertPixelToMM(SHOE_ALIGN_TARGET_PX_X, SHOE_ALIGN_TARGET_PX_Y, 
                                 final_target_mm_x, final_target_mm_y);
    
    float final_delta_x_gripper = final_target_mm_x;
    float final_delta_y_gripper = final_target_mm_y;

    MotorControlParams align_params = {10, 5, DEFAULT_P_GAIN, 0};
    motion_planner_.executeKinematicsSequence(
        final_target_mm_x, final_target_mm_y, 
        final_delta_x_gripper, final_delta_y_gripper, 
        align_params, gripper_state_
    );
    delay(DELAY_ALIGNMENT_MS);
    
    // 정렬 후 모터 5, 6번의 미세 조정
    MotorControlParams fine_tune_params = {20, 5, DEFAULT_P_GAIN, 0};
    motor_controller_.moveMotor(MotorID::MOTOR_ID_5, 10.0f, fine_tune_params);            
    motor_controller_.moveMotor(MotorID::MOTOR_ID_6, 0.0f, fine_tune_params);            

    comm_manager_.debugPrintln("Performing Linear Push (Return)...");
    float return_hook_x, return_hook_y;
    motion_planner_.calculateReturnPosition(
        final_target_mm_x, final_target_mm_y,
        final_delta_x_gripper, final_delta_y_gripper,
        return_hook_x, return_hook_y
    );
    
    delay(DELAY_ALIGNMENT_MS);
    MotorControlParams linear_params = {50, 20, DEFAULT_P_GAIN, 0};
    motion_planner_.executeLinearMotion(
        final_target_mm_x, final_target_mm_y, 
        return_hook_x, return_hook_y, 
        final_delta_x_gripper, final_delta_y_gripper, 
        LINEAR_MOTION_STEPS, linear_params, gripper_state_
    );
    delay(DELAY_ALIGNMENT_MS);
}

void RobotSequence::returnHome() {
    // 4. 모든 모터 초기 위치 복귀 루프
    comm_manager_.debugPrintln("Returning all motors to initial positions...");
    MotorControlParams return_params = {DEFAULT_VELOCITY, DEFAULT_ACCELERATION, DEFAULT_P_GAIN, DEFAULT_D_GAIN};
    const uint8_t return_move_ids[] = {MotorID::MOTOR_ID_2, MotorID::MOTOR_ID_3, MotorID::MOTOR_ID_4, MotorID::MOTOR_ID_5, MotorID::MOTOR_ID_6};

    for(uint8_t id : return_move_ids) {
        motion_planner_.moveToInitialPosition(id, return_params);
        delay(DELAY_MOTOR_INIT_MS);
    }

    comm_manager_.debugPrintln("Waiting before Motor1 return...");
    delay(DELAY_MOTOR1_WAIT_MS);

    // 마지막으로 1번 모터 복귀
    if(motor_controller_.pingMotor(MotorID::MOTOR_ID_1)) {
        MotorControlParams motor1_return_params = {10, 5, DEFAULT_P_GAIN, DEFAULT_D_GAIN};
        motor_controller_.moveMotor(MotorID::MOTOR_ID_1, -10.0f, motor1_return_params);
    }
    delay(DELAY_ALIGNMENT_MS);
    comm_manager_.debugPrintln("Sequence Complete.");
}

void RobotSequence::emergencyStop() {
    comm_manager_.debugPrintln("Emergency Stop: Turning off all motors...");
    for(uint8_t id : ALL_MOTOR_IDS) {
        motor_controller_.torqueOff(id); 
    }
    comm_manager_.debugPrintln("Torque OFF complete. Safe to close monitor.");
}

void RobotSequence::setStandardEncoders() {
    comm_manager_.debugPrintln("Setting Standard Encoders for ID 1 and 6...");
    uint8_t standard_setting_motor_ids[] = {MotorID::MOTOR_ID_1, MotorID::MOTOR_ID_6};
    for(uint8_t id : standard_setting_motor_ids) {
        if(motor_controller_.pingMotor(id)) {
            int32_t current_position = dxl.getPresentPosition(id);
            motor_controller_.setStandardEncoder(id, current_position);
        } else {
            comm_manager_.debugPrint("Motor ");
            comm_manager_.debugPrint(String(id));
            comm_manager_.debugPrintln(" PING FAIL (Cannot set standard encoder)");
        }
    }
}

void RobotSequence::initializeMotors() {
    moveMotorsToStartPosition();
}

void RobotSequence::moveMotorsToStartPosition() {
    comm_manager_.debugPrintln("Initializing Motors 2-6 to Start Position...");
    MotorControlParams init_params = {DEFAULT_VELOCITY, DEFAULT_ACCELERATION, DEFAULT_P_GAIN, DEFAULT_D_GAIN};
    const uint8_t initial_move_ids[] = {MotorID::MOTOR_ID_2, MotorID::MOTOR_ID_3, MotorID::MOTOR_ID_4, MotorID::MOTOR_ID_5, MotorID::MOTOR_ID_6};

    for(uint8_t id : initial_move_ids) {
        comm_manager_.debugPrint("Checking Motor ");
        comm_manager_.debugPrint(String(id));
        comm_manager_.debugPrint(" ... ");

        if(!motor_controller_.pingMotor(id)) {
            comm_manager_.debugPrintln("PING FAIL");
            continue;
        }
        comm_manager_.debugPrintln("PING OK");

        motor_controller_.initializeMotor(id, DEFAULT_CURRENT_LIMIT);
        motion_planner_.moveToInitialPosition(id, init_params);
        delay(DELAY_MOTOR_INIT_MS);
    }
}

void RobotSequence::initializeMotor1() {
    comm_manager_.debugPrintln("Waiting before Motor1...");
    delay(DELAY_MOTOR1_WAIT_MS);

    comm_manager_.debugPrintln("Moving Motor1 to Start Position...");
    if(motor_controller_.pingMotor(MotorID::MOTOR_ID_1)) {
        motor_controller_.initializeMotor(MotorID::MOTOR_ID_1, DEFAULT_CURRENT_LIMIT);
        MotorControlParams motor1_params = {10, 5, DEFAULT_P_GAIN, DEFAULT_D_GAIN};
        motor_controller_.moveMotor(MotorID::MOTOR_ID_1, -10.0f, motor1_params);
    } else {
        comm_manager_.debugPrintln("Motor1 PING FAIL");
    }
    delay(DELAY_SETUP_COMPLETE_MS);
}