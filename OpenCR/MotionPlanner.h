#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <Arduino.h>
#include "Configuration.h"
#include "RobotState.h"
#include "Kinematics.h"
#include "MotorController.h"

class MotionPlanner {
public:
    // 생성자
    MotionPlanner(Kinematics& kinematics, MotorController& motor_controller);
    
    // 기구학적 동작 실행
    void executeKinematicsSequence(
        float end_effector_x, float end_effector_y,
        float gripper_dx, float gripper_dy,
        const MotorControlParams& params,
        const GripperState& gripper_state,
        bool apply_delay_after_motor_move = true
    );
    
    // 선형 운동 실행
    void executeLinearMotion(
        float start_x, float start_y, 
        float end_x, float end_y,
        float gripper_dx, float gripper_dy,
        int num_steps,
        const MotorControlParams& params,
        const GripperState& gripper_state
    );
    
    // 훅 위치 계산
    void calculateHookPosition(
        float center_x, float center_y,
        float gripper_dx, float gripper_dy,
        float& hook_x, float& hook_y
    );
    
    // 반환 위치 계산
    void calculateReturnPosition(
        float target_x, float target_y,
        float gripper_dx, float gripper_dy,
        float& return_x, float& return_y
    );
    
    // 초기 위치로 모터 이동
    void moveToInitialPosition(uint8_t motor_id, const MotorControlParams& params);
    
private:
    Kinematics& kinematics_;
    MotorController& motor_controller_;
    
    // 벡터 정규화
    float normalizeVector(float dx, float dy);
};

#endif // MOTION_PLANNER_H