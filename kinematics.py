import math
from dataclasses import dataclass
from typing import Optional

# --- Kinematics Constants (pixel) ---
AX, AY = 1092, 384.6      # Winch anchor point
BX, BY = 1092, 121.7      # Shoulder joint rotation axis
L1 = 576.9               # Link 1 length
L2 = 512.8               # Link 2 length
FY_CONST = 12.8          # Fixed Target Height (y)
REF_AX, REF_AZ = AX, 866.3   # Base rotation pivot point

@dataclass
class KinematicsResult:
    theta0: float       # Base Rotation
    vec: float          # Orientation (rad)
    L4: float           # Winch Length
    theta2prime: float  # Shoulder Angle
    theta4: float       # Elbow Angle

class ArmKinematics:
    def solve(self, fx: float, fz: float, vec: float) -> Optional[KinematicsResult]:
        """
        입력: 목표 x, 목표 z, 목표 방향(vec, 라디안)
        출력: KinematicsResult (5개 변수) 또는 도달 불가 시 None
        """
        # 1. Base Rotation (Theta 0)
        dx_base = fx - REF_AX
        dz_base = fz - REF_AZ
        theta0_rad = math.atan2(-dx_base, -dz_base) 
        theta0 = 120 - math.degrees(theta0_rad) #~156.3

        # 2. 3D Distance Calculation
        dy_base = FY_CONST - BY
        L3 = math.sqrt(dx_base**2 + dy_base**2 + dz_base**2)

        # 작업 반경 확인
        if L3 > (L1 + L2) or L3 < abs(L1 - L2) or L3 == 0:
            return None

        try:
            # 3. Inverse Kinematics (Law of Cosines)
            # 3-1. Elbow Angle (Theta 4)
            cos_th4 = (L1**2 + L2**2 - L3**2) / (2 * L1 * L2)
            if cos_th4 > 1 or cos_th4 < -1:
                return None
            th4_rad = math.acos(cos_th4)

            # 3-2. Shoulder Angle (Theta 2)
            beta = math.atan2(L2, L3)
            sin_th2 = beta * math.sin(th4_rad)
            if sin_th2 > 1 or sin_th2 < -1:
                return None
            th2_prime_rad = math.asin(sin_th2)

            # 4. Winch Length (L4) Calculation
            term_width = L1 * math.cos(th2_prime_rad)
            term_height = (L1 * math.sin(th2_prime_rad)) - AY + BY
            L4 = math.sqrt(term_width**2 + term_height**2)

        except ValueError:
            return None

        return KinematicsResult(
            theta0=theta0,
            vec=vec,
            L4=L4,
            theta2prime=math.degrees(th2_prime_rad),
            theta4=math.degrees(th4_rad)
        )

# --- 테스트 실행용 ---
if __name__ == "__main__":
    solver = ArmKinematics()
    
    result = solver.solve(1271, 621, 0)
    
    if result:
        print(f"Calculated 5 Variables:")
        print(f"1. Theta0 (Base): {result.theta0:.2f}")
        print(f"2. L4 (Winch):    {result.L4:.2f}")
        print(f"3. Theta2' (Shld):{result.theta2prime:.2f}")
        print(f"4. Theta4 (Elbow):{result.theta4:.2f}")
        print(f"5. Vec (Wrist):   {math.degrees(result.vec):.2f}")
    else:
        print("Target unreachable.")