from dataclasses import dataclass

@dataclass(frozen=True)
class SpeedMode:
    v_max: float  # m/s
    w_max: float  # rad/s

@dataclass(frozen=True)
class TeleopParams:
    mode1 = SpeedMode(v_max=0.25, w_max=0.40) 
    mode2 = SpeedMode(v_max=0.55, w_max=1.50) 
    start_mode: int = 1 
    accel_lin: float = 0.6  
    accel_ang: float = 1.5   
    decel_lin: float = 0.8   
    decel_ang: float = 1.5    

    publish_rate_hz: float = 30.0 
    timeout_sec: float = 0.5
    hold_window_sec: float = 0.12
    deadband: float = 1e-3

def get_params():
    return TeleopParams()


