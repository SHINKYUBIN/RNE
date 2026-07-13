#include "CommunicationManager.h"

CommunicationManager::CommunicationManager() {
    // 생성자 - 특별한 초기화 없음
}

void CommunicationManager::initialize() {
    // 시리얼 통신 초기화
    DEBUG_SERIAL.begin(RPI_BAUDRATE);
    PI_SERIAL.begin(RPI_BAUDRATE);
}

bool CommunicationManager::hasIncomingData() {
    if (PI_SERIAL.available() > 0) {
        readInputData();
        return true;
    }
    return false;
}

bool CommunicationManager::isEmergencyStop() {
    return input_data_.equalsIgnoreCase("q");
}

bool CommunicationManager::parseShoeTarget(ShoeTarget& target) {
    // 라즈베리파이로부터 "shoe_id cx cy sx sy" 형식의 데이터를 파싱합니다.
    int parsed_items = sscanf(input_data_.c_str(), "%31s %f %f %f %f", 
                              target.identifier, &target.pixel_center_x, &target.pixel_center_y,
                              &target.pixel_source_x, &target.pixel_source_y);
    
    if (parsed_items >= 5) {
        target.is_valid = true;
        return true;
    } else {
        target.is_valid = false;
        debugPrintln("Error: Invalid data format received from Raspberry Pi.");
        return false;
    }
}

void CommunicationManager::debugPrint(const String& message) {
    DEBUG_SERIAL.print(message);
}

void CommunicationManager::debugPrintln(const String& message) {
    DEBUG_SERIAL.println(message);
}

void CommunicationManager::readInputData() {
    input_data_ = PI_SERIAL.readStringUntil('\n');
    input_data_.trim();
}