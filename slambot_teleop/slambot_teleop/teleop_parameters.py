from dataclasses import dataclass

@dataclass(frozen=True)
class SpeedMode:
    v_max: float  # m/s
    w_max: float  # rad/s

@dataclass(frozen=True)
class TeleopParams:
    mode1 = SpeedMode(v_max=2.0, w_max=1.8)
    mode2 = SpeedMode(v_max=4.0, w_max=2.0)
    start_mode: int = 2
    accel_lin: float = 1.2   # m/s^2
    accel_ang: float = 3.6   # rad/s^2
    decel_lin: float = 1.5   # m/s^2
    decel_ang: float = 4.0   # rad/s^2

    publish_rate_hz: float = 30.0
    timeout_sec: float = 0.5       
    hold_window_sec: float = 0.12   
    deadband: float = 1e-3        

def get_params():
    return TeleopParams()
