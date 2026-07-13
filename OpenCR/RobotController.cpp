#include "RobotController.h"

RobotController::RobotController(
    Kinematics& kinematics,
    MotorController& motor_controller
) : motion_planner_(kinematics, motor_controller),
    robot_sequence_(motion_planner_, kinematics, motor_controller, comm_manager_),
    current_state_(RobotState::IDLE) {
}

void RobotController::initialize() {
    // 통신 초기화
    comm_manager_.initialize();
    
    // 로봇 시퀀스 초기화 (모터 초기화 포함)
    robot_sequence_.initialize();
    
    setState(RobotState::IDLE);
}

void RobotController::update() {
    // 비상 정지 확인
    if (comm_manager_.hasIncomingData() && comm_manager_.isEmergencyStop()) {
        robot_sequence_.emergencyStop();
        return;
    }
    
    // 상태별 처리
    switch (current_state_) {
        case RobotState::IDLE:
            handleIdleState();
            break;
        case RobotState::MOVING_TO_SHOE:
            handleMovingToShoeState();
            break;
        case RobotState::HOOKING:
            handleHookingState();
            break;
        case RobotState::ALIGNING:
            handleAligningState();
            break;
        case RobotState::RETURNING_HOME:
            handleReturningHomeState();
            break;
        case RobotState::ERROR:
            handleErrorState();
            break;
    }
}

bool RobotController::receiveTarget() {
    if (comm_manager_.hasIncomingData()) {
        if (comm_manager_.parseShoeTarget(current_target_)) {
            return true;
        }
    }
    return false;
}

void RobotController::pickShoe() {
    setState(RobotState::MOVING_TO_SHOE);
    robot_sequence_.pickShoe(current_target_);
    setState(RobotState::HOOKING);
}

void RobotController::alignShoe() {
    setState(RobotState::ALIGNING);
    robot_sequence_.alignShoe();
}

void RobotController::returnHome() {
    setState(RobotState::RETURNING_HOME);
    robot_sequence_.returnHome();
    setState(RobotState::IDLE);
}

void RobotController::setState(RobotState new_state) {
    current_state_ = new_state;
}

void RobotController::handleIdleState() {
    if (receiveTarget()) {
        pickShoe();
    }
}

void RobotController::handleMovingToShoeState() {
    // pickShoe() 함수에서 상태 전환이 이미 처리됨
}

void RobotController::handleHookingState() {
    alignShoe();
}

void RobotController::handleAligningState() {
    returnHome();
}

void RobotController::handleReturningHomeState() {
    // returnHome() 함수에서 상태 전환이 이미 처리됨
}

void RobotController::handleErrorState() {
    // 에러 상태 처리 - 현재는 단순히 IDLE로 복귀
    setState(RobotState::IDLE);
}