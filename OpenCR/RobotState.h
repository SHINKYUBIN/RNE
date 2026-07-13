#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Arduino.h>

// 로봇의 상태를 정의하는 열거형
enum class RobotState {
    IDLE,                // 대기 상태
    MOVING_TO_SHOE,      // 신발 위치로 이동 중
    HOOKING,             // 신발 훅킹 중
    ALIGNING,            // 신발 정렬 중
    RETURNING_HOME,      // 초기 위치로 복귀 중
    ERROR                // 오류 상태
};

// 그리퍼 모터 상태를 관리하는 구조체
struct GripperState {
    bool motor_five_active;
    bool motor_six_active;
    
    GripperState() : motor_five_active(true), motor_six_active(true) {}
};

// 신발 타겟 정보를 저장하는 구조체
struct ShoeTarget {
    char identifier[32];
    float pixel_center_x, pixel_center_y;
    float pixel_source_x, pixel_source_y;
    float real_center_x, real_center_y;
    float real_source_x, real_source_y;
    float gripper_dx, gripper_dy;
    
    bool is_valid;
    
    ShoeTarget() : is_valid(false) {}
};

#endif // ROBOT_STATE_H