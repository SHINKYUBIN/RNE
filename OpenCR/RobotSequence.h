#ifndef ROBOT_SEQUENCE_H
#define ROBOT_SEQUENCE_H

#include <Arduino.h>
#include "Configuration.h"
#include "RobotState.h"
#include "MotionPlanner.h"
#include "Kinematics.h"
#include "MotorController.h"
#include "CommunicationManager.h"

class RobotSequence {
public:
    // 생성자
    RobotSequence(
        MotionPlanner& motion_planner,
        Kinematics& kinematics,
        MotorController& motor_controller,
        CommunicationManager& comm_manager
    );
    
    // 시스템 초기화
    void initialize();
    
    // 신발 픽업 시퀀스
    void pickShoe(const ShoeTarget& target);
    
    // 신발 정렬 시퀀스
    void alignShoe();
    
    // 초기 위치로 복귀
    void returnHome();
    
    // 비상 정지
    void emergencyStop();
    
private:
    MotionPlanner& motion_planner_;
    Kinematics& kinematics_;
    MotorController& motor_controller_;
    CommunicationManager& comm_manager_;
    
    GripperState gripper_state_;
    ShoeTarget current_target_;
    
    // 모터 초기화 시퀀스
    void initializeMotors();
    
    // 표준 인코더 설정
    void setStandardEncoders();
    
    // 모터를 초기 자세로 이동
    void moveMotorsToStartPosition();
    
    // Motor1 초기화 (별도 처리)
    void initializeMotor1();
};

#endif // ROBOT_SEQUENCE_H