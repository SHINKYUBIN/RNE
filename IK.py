import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

# ============================================================================
# [SECTION 1] 설정 및 상수 정의 (사용자 전체 코드 기준)
# ============================================================================
# --- 좌표 설정 ---
# 사용자의 요청: "회전점이 더 아래에 있다"
# 이미지 좌표계에서 Y값이 클수록 아래쪽입니다.
# 따라서 Y=384.6인 A가 회전점(Pivot), Y=121.7인 B가 윈치(Anchor)가 됩니다.

# A: 회전점 (Pivot/Shoulder) - 아래쪽
AX, AY = 1092.0, 384.6  
# B: 윈치 고정점 (Winch Anchor) - 위쪽
BX, BY = 1092.0, 121.7  

# --- 링크 길이 (단위: pixel) ---
L1 = 576.9   # 상박 (Arm)
L2 = 512.8   # 하박 (Forearm)

# --- 초기 목표값 설정 ---
INIT_DIST = 500.0   # A(Pivot)로부터의 수평 거리
INIT_FY = 100.0     # 목표 높이 Y (위쪽을 향하도록 설정)

# ============================================================================
# [SECTION 2] 기구학 연산 함수 (A가 회전점)
# ============================================================================
def calculate_ik_final(target_dist_x, target_y):
    """
    입력:
      target_dist_x: 회전점(A)으로부터 목표물까지의 수평 거리 (+방향)
      target_y: 목표물의 Y 좌표 (높이)
    출력:
      th_shoulder: 어깨 회전각 (Global)
      elbow_pos: 팔꿈치 좌표 (Ex, Ey)
      L3: A와 목표점 사이 거리 (Pivot -> Target)
      L4: B와 팔꿈치 사이 거리 (Winch -> Elbow)
    """
    
    # 1. 목표점 좌표 (F) 계산
    # A(회전점)를 기준으로 target_dist_x 만큼 오른쪽
    fx = AX + target_dist_x
    fy = target_y

    # 2. L3 계산 (A에서 F까지의 거리) [Pivot -> Target]
    dx = fx - AX
    dy = fy - AY
    L3 = math.sqrt(dx**2 + dy**2)

    # 3. 도달 가능 여부 확인
    if L3 > (L1 + L2) or L3 < abs(L1 - L2) or L3 == 0:
        return None, None, L3, None, False

    # 4. 관절 각도 계산 (코사인 제 2법칙)
    # 삼각형 A-E-F (변: L1, L2, L3)
    
    # alpha: L3 선분과 L1 선분 사이의 내각 (A 지점의 내각)
    # L2^2 = L1^2 + L3^2 - 2*L1*L3*cos(alpha)
    cos_alpha = (L1**2 + L3**2 - L2**2) / (2 * L1 * L3)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha_rad = math.acos(cos_alpha)
    
    # beta: L3 선분의 글로벌 각도 (수평선 기준)
    beta_rad = math.atan2(dy, dx)
    
    # 전체 어깨 각도 (theta_shoulder)
    # "Elbow Up" 형상 (팔꿈치가 L3 선분보다 위쪽에 위치)
    # 이미지 좌표계(Y 아래로 증가)에서 '위쪽'은 각도가 더 작아지는(빼는) 방향
    th_shoulder = beta_rad - alpha_rad
    
    # 5. 팔꿈치(Elbow, E) 위치 계산
    # A(Pivot)에서 시작
    ex = AX + L1 * math.cos(th_shoulder)
    ey = AY + L1 * math.sin(th_shoulder)
    
    # 6. L4 계산 (B에서 E까지의 거리) [Winch -> Elbow]
    dist_bx = ex - BX
    dist_by = ey - BY
    L4 = math.sqrt(dist_bx**2 + dist_by**2)

    return th_shoulder, (ex, ey), L3, L4, True

# ============================================================================
# [SECTION 3] 시각화 (Matplotlib)
# ============================================================================
fig, ax = plt.subplots(figsize=(10, 8))
plt.subplots_adjust(left=0.1, bottom=0.25)

# 좌표계 설정 (Y축 반전: 이미지가 아래로 갈수록 좌표 증가)
ax.invert_yaxis()
ax.set_aspect('equal')
ax.grid(True)
ax.set_title(f"Sim: Pivot=A(Bottom, {AY}), Winch=B(Top, {BY})")
ax.set_xlabel("X Coordinate (pixel)")
ax.set_ylabel("Y Coordinate (pixel)")

# 범위 설정 (여유 있게)
ax.set_xlim(AX - 200, AX + L1 + L2 + 100)
ax.set_ylim(AY + 200, BY - 200) # Y축 반전되어 있으므로 순서 주의

# --- 플롯 요소 ---
# 고정점 A, B
point_A, = ax.plot([AX], [AY], 'bo', markersize=12, label='A (Pivot/Shoulder)')
point_B, = ax.plot([BX], [BY], 'ks', markersize=10, label='B (Winch Anchor)')

# 링크 및 라인
line_l1, = ax.plot([], [], 'b-', linewidth=5, label='L1 (Arm)')
line_l2, = ax.plot([], [], 'r-', linewidth=5, label='L2 (Forearm)')
line_l3, = ax.plot([], [], 'g--', linewidth=1, label='L3 (A -> Target)') # Pivot -> Target
line_l4, = ax.plot([], [], 'm:', linewidth=3, label='L4 (B -> Elbow)')  # Winch -> Elbow

# 움직이는 점
point_elbow, = ax.plot([], [], 'go', markersize=8)
point_target, = ax.plot([], [], 'rx', markersize=12, markeredgewidth=3, label='Target')

# 텍스트 정보
info_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

# --- 업데이트 함수 ---
def update(val):
    dist_x = slider_dist.val
    target_y = slider_fy.val
    
    th_shoulder, elbow_pos, L3, L4, reachable = calculate_ik_final(dist_x, target_y)
    
    # 타겟 좌표
    fx = AX + dist_x
    fy = target_y
    point_target.set_data([fx], [fy])

    if not reachable:
        info_text.set_text("Target Out of Reach!")
        line_l1.set_data([], [])
        line_l2.set_data([], [])
        line_l3.set_data([], [])
        line_l4.set_data([], [])
        fig.canvas.draw_idle()
        return

    ex, ey = elbow_pos

    # 라인 그리기
    line_l1.set_data([AX, ex], [AY, ey])      # A -> Elbow
    line_l2.set_data([ex, fx], [ey, fy])      # Elbow -> Target
    line_l3.set_data([AX, fx], [AY, fy])      # A -> Target
    line_l4.set_data([BX, ex], [BY, ey])      # B -> Elbow
    
    point_elbow.set_data([ex], [ey])
    
    # 정보 텍스트
    info = (f"Target (Fx, Fy): ({fx:.1f}, {fy:.1f})\n"
            f"L3 (Pivot A->Target): {L3:.2f}\n"
            f"L4 (Winch B->Elbow): {L4:.2f}\n"
            f"Theta (Shoulder): {math.degrees(th_shoulder):.2f}°")
    info_text.set_text(info)
    
    fig.canvas.draw_idle()

# --- 슬라이더 UI ---
ax_dist = plt.axes([0.2, 0.1, 0.65, 0.03])
ax_fy = plt.axes([0.2, 0.05, 0.65, 0.03])

slider_dist = Slider(ax_dist, 'Horiz Dist (from A)', 100.0, L1+L2, valinit=INIT_DIST)
# Y값 슬라이더: 화면 위쪽(작은 Y)으로 갈 수 있도록 범위 설정
slider_fy = Slider(ax_fy, 'Target Y', 0.0, AY + 100, valinit=INIT_FY)

slider_dist.on_changed(update)
slider_fy.on_changed(update)

update(None)
plt.show()