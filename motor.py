import time
from adafruit_servokit import ServoKit
from gpiozero import Motor

kit = ServoKit(channels=16)
kit.frequency = 60

print("start")

# 채널 설정
TARGET_CHANNEL1 = 0
TARGET_CHANNEL2 = 1
TARGET_CHANNEL3 = 2
TARGET_CHANNEL4 = 3 # s4
TARGET_CHANNEL5 = 4 # s5
TARGET_CHANNEL6 = 5
TARGET_CHANNEL7 = 6

motor1 = Motor(forward = 17, backward = 18)
motor2 = Motor(forward = 22, backward = 23)

# --- [추가됨] 보정 함수 ---
def set_corrected_angle(channel, target_angle):
    """
    원하는 각도(target_angle)를 입력하면 0.78배 감소하는 현상을 고려하여
    보정된 각도를 계산해 서보에 입력합니다.
    """
    try:
        # 보정 계산: 목표 각도 / 0.78
        corrected_angle = target_angle / 0.78
        
        # 안전 장치: 180도를 넘거나 0도보다 작으면 제한
        if target_angle > 180:
            corrected_angle = 180
        elif target_angle < 0:
            corrected_angle = 0
            
        kit.servo[channel].angle = corrected_angle
        # 디버깅용 출력 (필요 시 주석 해제)
        # print(f"Target: {target_angle}, Sent: {corrected_angle:.2f}")
        
    except Exception as e:
        print(f"Servo Error on ch {channel}: {e}")
# -----------------------

try:
    print("90")
    print("st")
    
    kit.servo[TARGET_CHANNEL1].angle = 156.3
    kit.servo[TARGET_CHANNEL2].angle = 180
    kit.servo[TARGET_CHANNEL3].angle = 0
    
    time.sleep(1)
    motor1.stop()
    
    # --- [수정됨] 문제가 있는 s4, s5에 보정 함수 적용 ---
    
    
    # 새로운 방식 (s4, s5 적용 예시)
    set_corrected_angle(TARGET_CHANNEL4, 47) 
    set_corrected_angle(TARGET_CHANNEL5, 93.6) # 120도 * 0.78 = 93.6 이므로, 120도를 원하면 120 입력
    
    # 테스트용 (원하는 각도를 여기에 넣으세요)
    set_corrected_angle(TARGET_CHANNEL4, 47)
    
    # kit.servo[TARGET_CHANNEL6].angle = 0
    kit.servo[TARGET_CHANNEL7].angle = 180
     
    time.sleep(5)
    
    # motor1.backward()
    motor1.stop()
    motor2.stop()
    
    # 각도 초기화 (None으로 설정하여 힘 풀기)
    kit.servo[TARGET_CHANNEL1].angle = None
    kit.servo[TARGET_CHANNEL2].angle = None
    kit.servo[TARGET_CHANNEL3].angle = None
    kit.servo[TARGET_CHANNEL4].angle = None
    kit.servo[TARGET_CHANNEL5].angle = None
    kit.servo[TARGET_CHANNEL6].angle = None
    kit.servo[TARGET_CHANNEL7].angle = None
    
    print("test")

except Exception as e:
    print(f"error{e}")