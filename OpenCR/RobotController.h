#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino.h>
#include "Configuration.h"
#include "RobotState.h"
#include "RobotSequence.h"
#include "MotionPlanner.h"
#include "Kinematics.h"
#include "MotorController.h"
#include "CommunicationManager.h"

class RobotController {
public:
    // 생성자
    RobotController(
        Kinematics& kinematics,
        MotorController& motor_controller
    );
    
    // 시스템 초기화
    void initialize();
    
    // 메인 업데이트 루프
    void update();
    
    // 타겟 수신 확인
    bool receiveTarget();
    
    // 신발 픽업
    void pickShoe();
    
    // 신발 정렬
    void alignShoe();
    
    // 초기 위치로 복귀
    void returnHome();
    
private:
    // 모듈 인스턴스들
    CommunicationManager comm_manager_;
    MotionPlanner motion_planner_;
    RobotSequence robot_sequence_;
    
    // 현재 상태
    RobotState current_state_;
    ShoeTarget current_target_;
    
    // 상태 전환
    void setState(RobotState new_state);
    
    // 상태별 처리
    void handleIdleState();
    void handleMovingToShoeState();
    void handleHookingState();
    void handleAligningState();
    void handleReturningHomeState();
    void handleErrorState();
};

#endif // ROBOT_CONTROLLER_H