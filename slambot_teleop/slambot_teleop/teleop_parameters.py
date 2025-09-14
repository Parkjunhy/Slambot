from dataclasses import dataclass

@dataclass(frozen=True)
class SpeedMode:
    v_max: float  # m/s
    w_max: float  # rad/s

@dataclass(frozen=True)
class TeleopParams:
    # 매핑 안전 모드(기본)
    mode1 = SpeedMode(v_max=0.30, w_max=0.60)
    # 약간 빠른 모드(여유 공간에서)
    mode2 = SpeedMode(v_max=0.50, w_max=0.90)

    start_mode: int = 1  # 안전 모드부터 시작

    # 급가속/급회전 억제(모션 블러·슬립 완화)
    accel_lin: float = 0.6    # m/s^2
    accel_ang: float = 1.8    # rad/s^2
    decel_lin: float = 0.8
    decel_ang: float = 2.2

    publish_rate_hz: float = 30.0
    timeout_sec: float = 0.5
    hold_window_sec: float = 0.12
    deadband: float = 1e-3

def get_params():
    return TeleopParams()
