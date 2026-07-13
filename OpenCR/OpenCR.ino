#include <Dynamixel2Arduino.h>
#include "Configuration.h"
#include "Kinematics.h"
#include "MotorController.h"
#include "RobotController.h"

// Dynamixel2Arduino 객체 선언 (통신 포트와 Direction Pin)
Dynamixel2Arduino dxl(DXL_SERIAL, 84);

// 모듈 인스턴스화
Kinematics kinematics_solver(
    LINK_LENGTH_1, LINK_LENGTH_2, LINK_LENGTH_3,
    LINK_LENGTH_4, GRIPPER_OFFSET_RADIUS
);
MotorController motor_controller(dxl);
RobotController robot(kinematics_solver, motor_controller);

void setup() {
    robot.initialize();
}

void loop() {
    robot.update();
}