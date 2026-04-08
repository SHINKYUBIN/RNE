import socket
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import time

# --- 설정값 ---
ESP_IP = '10.172.144.69'  # ESP32 IP 주소
ESP_PORT = 8888           # ESP32 포트
SAMPLE_RATE = 3200        # ADXL345 샘플링 레이트 (Hz)
BUFFER_SIZE = 2048        # 화면에 보여줄 데이터 개수
FFT_SIZE = 2048           # FFT 분석 구간 크기

# 데이터 저장용 덱 (Deque)
data_buffer = deque(maxlen=BUFFER_SIZE)
timestamps = deque(maxlen=BUFFER_SIZE)

# 데이터 수신 상태 플래그
is_running = True

def read_stream():
    """소켓 통신을 통해 ESP32 데이터를 백그라운드에서 수신하는 함수"""
    global is_running
    
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(5)
        print(f"Connecting to {ESP_IP}:{ESP_PORT}...")
        client_socket.connect((ESP_IP, ESP_PORT))
        print("Connected! Receiving data...")
        
        buffer_str = ""
        
        while is_running:
            try:
                # 데이터 수신 (청크 단위)
                data = client_socket.recv(4096).decode('utf-8', errors='ignore')
                if not data:
                    break
                
                buffer_str += data
                
                # 줄바꿈 문자로 데이터 분리
                while '\n' in buffer_str:
                    line, buffer_str = buffer_str.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                        
                    try:
                        # ESP32 포맷: Time(ms),X,Y,Z
                        parts = line.split(',')
                        if len(parts) == 4:
                            t = int(parts[0])
                            x = float(parts[1])
                            y = float(parts[2])
                            z = float(parts[3])
                            
                            # 벡터 합성값 (SVM) 계산
                            svm = np.sqrt(x**2 + y**2 + z**2)
                            
                            data_buffer.append(svm)
                            timestamps.append(t)
                            
                    except ValueError:
                        continue # 파싱 에러 무시
                        
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Data Read Error: {e}")
                break
                
    except Exception as e:
        print(f"Connection Failed: {e}")
    finally:
        client_socket.close()
        is_running = False

def update_plot(frame, line_wave, line_fft, ax_wave, ax_fft):
    """애니메이션 업데이트 함수"""
    if len(data_buffer) < BUFFER_SIZE:
        return line_wave, line_fft

    # 1. 데이터 준비 (리스트 변환)
    y_data = np.array(data_buffer)
    
    # 2. 직류 성분(중력 가속도 등) 제거 - High Pass Filter 효과
    y_ac = y_data - np.mean(y_data)
    
    # --- 시간 영역 그래프 업데이트 ---
    x_data = np.arange(len(y_ac))
    line_wave.set_data(x_data, y_ac)
    
    # [수정됨] Y축 스케일 동적 조정 (민감도 향상)
    peak = np.max(np.abs(y_ac))
    current_ylim = ax_wave.get_ylim()[1]
    
    # 피크가 현재 범위보다 크면 늘림 (빠르게 반응)
    if peak > current_ylim:
        ax_wave.set_ylim(-peak * 1.1, peak * 1.1)
    # 피크가 현재 범위의 20% 미만이면 줄임 (천천히 줌인)
    elif peak < current_ylim * 0.2 and current_ylim > 0.2: 
        new_lim = current_ylim * 0.95
        ax_wave.set_ylim(-new_lim, new_lim)
    
    # --- 주파수 영역(FFT) 그래프 업데이트 ---
    if len(y_ac) >= FFT_SIZE:
        fft_data = y_ac[-FFT_SIZE:]
        
        # FFT 계산
        fft_vals = np.fft.fft(fft_data)
        fft_freqs = np.fft.fftfreq(FFT_SIZE, 1.0/SAMPLE_RATE)
        
        # 양의 주파수 대역만 사용
        pos_mask = fft_freqs > 0
        freqs = fft_freqs[pos_mask]
        mags = np.abs(fft_vals[pos_mask]) / FFT_SIZE # 정규화
        
        line_fft.set_data(freqs, mags)
        
        # [수정됨] FFT Y축 동적 조정 (민감도 향상)
        max_mag = np.max(mags) if len(mags) > 0 else 0.1
        current_fft_ylim = ax_fft.get_ylim()[1]
        
        if max_mag > current_fft_ylim:
            ax_fft.set_ylim(0, max_mag * 1.2)
        elif max_mag < current_fft_ylim * 0.5:
            ax_fft.set_ylim(0, max_mag * 1.5)

    return line_wave, line_fft

# --- 메인 실행부 ---
if __name__ == "__main__":
    # 데이터 수신 스레드 시작
    t = threading.Thread(target=read_stream)
    t.daemon = True
    t.start()
    
    # 그래프 설정
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # 1. 시간 영역 (Waveform)
    line_wave, = ax1.plot([], [], lw=1, color='blue')
    ax1.set_title(f"Real-time Vibration (SVM - Gravity Removed) @ {SAMPLE_RATE}Hz")
    ax1.set_xlabel("Samples")
    ax1.set_ylabel("Acceleration (m/s^2)")
    ax1.set_xlim(0, BUFFER_SIZE)
    
    # [수정됨] 초기 범위를 아주 좁게 설정하여 미세 진동 확인
    ax1.set_ylim(-0.2, 0.2) 
    ax1.grid(True)
    
    # 2. 주파수 영역 (FFT Spectrum)
    line_fft, = ax2.plot([], [], lw=1, color='red')
    ax2.set_title("Frequency Spectrum (FFT)")
    ax2.set_xlabel("Frequency (Hz)")
    ax2.set_ylabel("Magnitude")
    ax2.set_xlim(0, 200) 
    
    # [수정됨] 초기 범위를 아주 좁게 설정
    ax2.set_ylim(0, 0.05)
    ax2.grid(True)
    
    plt.tight_layout()
    
    # 애니메이션 실행 (interval=50ms 마다 갱신)
    ani = animation.FuncAnimation(fig, update_plot, fargs=(line_wave, line_fft, ax1, ax2),
                                  interval=50, blit=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        is_running = False