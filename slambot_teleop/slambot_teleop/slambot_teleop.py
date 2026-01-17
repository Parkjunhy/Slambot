import sys
import termios
import tty
import time
import select
import atexit
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from slambot_teleop.teleop_parameters import get_params, TeleopParams, SpeedMode

class SlambotTeleop(Node):
    def __init__(self):
        super().__init__('slambot_teleop')

        self.cfg: TeleopParams = get_params()
        self.mode = self.cfg.start_mode
        self.v = 0.0
        self.w = 0.0
        self.last_press = {'w': 0.0, 's': 0.0, 'a': 0.0, 'd': 0.0}
        self.last_any_key_time = time.time()

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dt = 1.0 / self.cfg.publish_rate_hz
        self.timer = self.create_timer(self.dt, self.update)

        self.fd = sys.stdin.fileno()
        self._old_tty = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        atexit.register(self.restore_terminal_settings)

        self.get_logger().info(self._help_text())

    def restore_terminal_settings(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old_tty)
        except Exception as e:
            self.get_logger().error(f"Failed to restore terminal settings: {e}")

    def _help_text(self) -> str:
        return (
            f"  Start : {self.mode}단, Publish {self.cfg.publish_rate_hz:.0f}Hz\n"
        )

    def _current_mode_limits(self) -> SpeedMode:
        return self.cfg.mode1 if self.mode == 1 else self.cfg.mode2

    def _read_keys_nonblock(self):
        updated = False
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            now = time.time()
            self.last_any_key_time = now
            updated = True

            if ch in ('w', 'a', 's', 'd'):
                self.last_press[ch] = now
            elif ch == '1':
                self.mode = 1
                self.get_logger().info(f"[MODE] 1단: v_max={self._current_mode_limits().v_max:.2f}, w_max={self._current_mode_limits().w_max:.2f}")
            elif ch == '2':
                self.mode = 2
                self.get_logger().info(f"[MODE] 2단: v_max={self._current_mode_limits().v_max:.2f}, w_max={self._current_mode_limits().w_max:.2f}")
            elif ch == ' ':
                self.v = 0.0
                self.w = 0.0
                self.get_logger().info("stop")
            elif ch in ('q', '\x03'):  
                raise KeyboardInterrupt
        return updated

    @staticmethod
    def _approach_zero(x: float, rate: float, dt: float, deadband: float) -> float:
        if abs(x) <= deadband:
            return 0.0
        sign = 1.0 if x > 0.0 else -1.0
        x -= sign * rate * dt
        return 0.0 if x * sign < 0.0 else x

    def update(self):
        try:
            self._read_keys_nonblock()
        except KeyboardInterrupt:
            rclpy.shutdown()
            return

        now = time.time()
        hold = self.cfg.hold_window_sec
        
        forward = (now - self.last_press['w']) <= hold
        backward = (now - self.last_press['s']) <= hold
        left = (now - self.last_press['a']) <= hold
        right = (now - self.last_press['d']) <= hold
        
        m = self._current_mode_limits()

        if forward and not backward:
            self.v = min(self.v + self.cfg.accel_lin * self.dt, m.v_max)
        elif backward and not forward:
            self.v = max(self.v - self.cfg.accel_lin * self.dt, -m.v_max)
        else:
            self.v = self._approach_zero(self.v, self.cfg.decel_lin, self.dt, self.cfg.deadband)

        if left and not right:
            self.w = min(self.w + self.cfg.accel_ang * self.dt, m.w_max)
        elif right and not left:
            self.w = max(self.w - self.cfg.accel_ang * self.dt, -m.w_max)
        else:
            self.w = self._approach_zero(self.w, self.cfg.decel_ang, self.dt, self.cfg.deadband)

        if (now - self.last_any_key_time) > self.cfg.timeout_sec:
            self.v = self._approach_zero(self.v, self.cfg.decel_lin, self.dt, self.cfg.deadband)
            self.w = self._approach_zero(self.w, self.cfg.decel_ang, self.dt, self.cfg.deadband)

        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SlambotTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()