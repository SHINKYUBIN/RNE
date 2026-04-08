"""
층간 소음 분석기 V3: 3-Axis Vector Sum Analysis (최종 수정)
- 해결: 데이터 파싱 개수 수정 (3개 -> 4개)
- 기능: X,Y,Z 축을 모두 합쳐서 어떤 방향의 진동도 놓치지 않음
"""
import socket
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import queue
import time
from scipy.signal import butter, lfilter

# --- 1. 설정값 ---
ESP_IP = '10.172.144.69'  # [확인] ESP32 IP가 맞는지 다시 확인해주세요!
PORT = 8888
FS = 3200                 # 샘플링 레이트
BUFFER_SIZE = 3200 * 2    # 2초간 데이터 표시

# 필터 설정
LOW_CUT = 10.0
HIGH_CUT = 500.0

# --- 2. 신호처리 클래스 ---
class SignalProcessor:
    def __init__(self, fs, low, high):
        self.fs = fs
        nyq = 0.5 * fs
        self.b, self.a = butter(2, [low/nyq, high/nyq], btype='band')

    def apply_filter(self, data_chunk):
        if len(data_chunk) < 10: return data_chunk
        return lfilter(self.b, self.a, data_chunk)

# --- 3. 데이터 수신 스레드 ---
data_queue = queue.Queue()

def receive_data_thread():
    print(f"Connecting to {ESP_IP}:{PORT}...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((ESP_IP, PORT))
        print("✅ Connected! Receiving 3-Axis data...")
        
        buffer_str = ""
        while True:
            try:
                chunk = s.recv(4096).decode('utf-8', errors='ignore')
                if not chunk: break
                
                buffer_str += chunk
                while '\n' in buffer_str:
                    line, buffer_str = buffer_str.split('\n', 1)
                    if not line: continue
                    
                    parts = line.split(',')
                    
                    # [수정된 부분] ESP32가 시간,X,Y,Z (4개)를 보내므로 4로 변경
                    if len(parts) == 4: 
                        try:
                            x = float(parts[1])
                            y = float(parts[2])
                            z = float(parts[3])
                            data_queue.put((x, y, z)) # 3축 데이터 큐에 넣기
                        except ValueError:
                            continue
            except Exception:
                break
    except Exception as e:
        print(f"Connection Failed: {e}")

# --- 4. 메인 루프 ---
def main():
    t = threading.Thread(target=receive_data_thread, daemon=True)
    t.start()
    
    processor = SignalProcessor(FS, LOW_CUT, HIGH_CUT)
    plot_data = deque([0.0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
    
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(12, 8))
    
    # 상단 그래프
    ax1 = fig.add_subplot(2, 1, 1)
    line1, = ax1.plot(np.zeros(BUFFER_SIZE), color='#00ffea', lw=1)
    ax1.set_title("Total Vibration Energy (Vector Sum)")
    ax1.set_ylim(-5.0, 5.0) # 범위 살짝 늘림
    
    # 하단 그래프
    ax2 = fig.add_subplot(2, 1, 2)
    f_axis = np.fft.rfftfreq(BUFFER_SIZE, 1/FS)
    line2, = ax2.plot(f_axis, np.zeros(len(f_axis)), color='#ff00ff', lw=1)
    ax2.set_title("Frequency Analysis")
    ax2.set_xlim(0, 500)
    ax2.set_ylim(0, 50)

    last_update_time = time.time()
    
    def update(frame):
        nonlocal last_update_time
        
        # 큐 데이터 꺼내기
        raw_x, raw_y, raw_z = [], [], []
        while not data_queue.empty():
            try:
                val = data_queue.get_nowait()
                raw_x.append(val[0])
                raw_y.append(val[1])
                raw_z.append(val[2])
            except queue.Empty:
                break
            if len(raw_x) > 2000: break 
            
        if not raw_x: return line1, line2

        # 1. 3축 벡터 합 계산 (모든 방향 충격 감지)
        np_x = np.array(raw_x)
        np_y = np.array(raw_y)
        np_z = np.array(raw_z)
        
        # 중력 제거 (간이): Z축 평균 빼기
        clean_z = np_z - np.mean(np_z)
        
        # 벡터 합 계산
        magnitude = np.sqrt(np_x**2 + np_y**2 + clean_z**2)
        
        # 2. 필터링
        filtered_mag = processor.apply_filter(magnitude)
        plot_data.extend(filtered_mag)
        
        # 화면 갱신 제한
        if time.time() - last_update_time < 0.03: return line1, line2
        last_update_time = time.time()
        
        # 3. 그리기
        data_to_plot = np.array(plot_data)
        line1.set_ydata(data_to_plot)
        
        # 충격 감지 시 FFT 갱신
        recent_max = np.max(np.abs(data_to_plot[-320:]))
        if recent_max > 0.1:
            fft_vals = np.abs(np.fft.rfft(data_to_plot * np.hanning(len(data_to_plot))))
            line2.set_ydata(fft_vals)
            ax2.set_ylim(0, max(50, np.max(fft_vals) * 1.2))
            
            if recent_max > 1.0:
                print(f"💥 쿵! 감지됨: {recent_max:.2f}")

        return line1, line2

    ani = FuncAnimation(fig, update, interval=30, blit=True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()