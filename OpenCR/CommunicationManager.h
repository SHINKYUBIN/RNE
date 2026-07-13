#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include <Arduino.h>
#include "Configuration.h"
#include "RobotState.h"

class CommunicationManager {
public:
    // 생성자
    CommunicationManager();
    
    // 통신 초기화
    void initialize();
    
    // 라즈베리파이로부터 데이터 수신 확인
    bool hasIncomingData();
    
    // 비상 정지 명령 확인
    bool isEmergencyStop();
    
    // 신발 타겟 데이터 파싱
    bool parseShoeTarget(ShoeTarget& target);
    
    // 디버그 메시지 출력
    void debugPrint(const String& message);
    void debugPrintln(const String& message);
    
private:
    String input_data_;
    
    // 입력 데이터 읽기
    void readInputData();
};

#endif // COMMUNICATION_MANAGER_H