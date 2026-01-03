from dataclasses import dataclass

@dataclass(frozen=True)
class SpeedMode:
    v_max: float  # m/s
    w_max: float  # rad/s

@dataclass(frozen=True)
class TeleopParams:
    # 모드 1: 정밀 매핑 모드 (천천히 꼼꼼하게)
    mode1 = SpeedMode(v_max=0.25, w_max=0.30) 
    # 모드 2: 일반 주행 모드 (자율 주행 시 적합)
    mode2 = SpeedMode(v_max=0.55, w_max=1.50) 
    start_mode: int = 1 # 안전 모드부터 시작
    # 가속도: 4륜의 접지력을 믿고 조금 더 기민하게 설정
    accel_lin: float = 0.8    # 0.6 -> 0.8로 상향 (반응성 개선)
    accel_ang: float = 1.5    # 1.8 -> 2.5로 상향 (회전 시작 시 답답함 해소)
    decel_lin: float = 0.8    # 급정거 능력 강화
    decel_ang: float = 1.5    

    publish_rate_hz: float = 30.0 # 라즈베리파이 4에서 적절한 부하
    # ... 나머지 동일
    timeout_sec: float = 0.5
    hold_window_sec: float = 0.12
    deadband: float = 1e-3

def get_params():
    return TeleopParams()


