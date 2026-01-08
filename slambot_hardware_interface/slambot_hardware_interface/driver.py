import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from dynamixel_sdk import *

LEFT_FRONT_ID  = 3
LEFT_REAR_ID   = 2
RIGHT_FRONT_ID = 4
RIGHT_REAR_ID  = 1

DEVICENAME       = '/dev/ttyACM0'
BAUDRATE         = 115200
PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_VELOCITY    = 104  
ADDR_PRESENT_POSITION = 132  

VELOCITY_CONTROL_MODE = 1    

WHEEL_SEPARATION = 0.15     
WHEEL_RADIUS     = 0.033   
VELOCITY_SCALE   = 500     

TICKS_PER_REV = 4096.0     

LEFT_DIR  = 1
RIGHT_DIR = -1


class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler, ADDR_GOAL_VELOCITY, 4
        )

        self.groupSyncRead = GroupSyncRead(
            self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, 4
        )

        if not self.portHandler.openPort():
            self.get_logger().error("포트를 열 수 없습니다.")
            sys.exit(1)

        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("BAUDRATE 설정 실패!")
            sys.exit(1)

        self.setup_motors()
        self.setup_sync_read()

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer_period = 0.033  
        self.timer = self.create_timer(self.timer_period, self.update_odometry)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_time = self.get_clock().now()

        self.is_initialized = False
        self.prev_ticks = {
            LEFT_FRONT_ID: 0, LEFT_REAR_ID: 0,
            RIGHT_FRONT_ID: 0, RIGHT_REAR_ID: 0
        }

        self.get_logger().info("WheelController (Pose+Twist) 준비 완료.")

    def setup_motors(self):
        ids = [LEFT_FRONT_ID, LEFT_REAR_ID, RIGHT_FRONT_ID, RIGHT_REAR_ID]
        for dxl_id in ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)

    def setup_sync_read(self):
        ids = [LEFT_FRONT_ID, LEFT_REAR_ID, RIGHT_FRONT_ID, RIGHT_REAR_ID]
        for dxl_id in ids:
            self.groupSyncRead.addParam(dxl_id)

    def update_odometry(self):
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            return  

        current_ticks = {}
        ids = [LEFT_FRONT_ID, LEFT_REAR_ID, RIGHT_FRONT_ID, RIGHT_REAR_ID]
        
        for dxl_id in ids:
            if self.groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, 4):
                val = self.groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, 4)
                if val > 0x7FFFFFFF: val -= 4294967296
                current_ticks[dxl_id] = val
            else:
                return  
            
        if not self.is_initialized:
            self.prev_ticks = current_ticks
            self.is_initialized = True
            self.get_logger().info(f"오도메트리 초기화 완료. 시작 틱: {current_ticks}")
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0: return

        d_fl = (current_ticks[LEFT_FRONT_ID]  - self.prev_ticks[LEFT_FRONT_ID]) * LEFT_DIR
        d_rl = (current_ticks[LEFT_REAR_ID]   - self.prev_ticks[LEFT_REAR_ID])  * (-1 * LEFT_DIR)
        d_fr = (current_ticks[RIGHT_FRONT_ID] - self.prev_ticks[RIGHT_FRONT_ID]) * RIGHT_DIR
        d_rr = (current_ticks[RIGHT_REAR_ID]  - self.prev_ticks[RIGHT_REAR_ID]) * (-1 * RIGHT_DIR) 

        self.prev_ticks = current_ticks

        m_per_tick = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV
        
        dist_fl = d_fl * m_per_tick
        dist_rl = d_rl * m_per_tick
        dist_fr = d_fr * m_per_tick
        dist_rr = d_rr * m_per_tick

        dist_left  = (dist_fl + dist_rl) / 2.0
        dist_right = (dist_fr + dist_rr) / 2.0

        d_center = (dist_left + dist_right) / 2.0
        d_theta  = (dist_right - dist_left) / WHEEL_SEPARATION

        self.x  += d_center * math.cos(self.th + d_theta / 2.0)
        self.y  += d_center * math.sin(self.th + d_theta / 2.0)
        self.th += d_theta

        v_linear  = d_center / dt
        v_angular = d_theta  / dt

        q = self.euler_to_quaternion(0, 0, self.th)
        current_ros_time = current_time.to_msg()

        t = TransformStamped()
        t.header.stamp = current_ros_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_ros_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        
        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.angular.z = v_angular

        self.odom_publisher.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        fl_speed = v - w * WHEEL_SEPARATION / 2.0
        fr_speed = v + w * WHEEL_SEPARATION / 2.0
        rl_speed = v - w * WHEEL_SEPARATION / 2.0
        rr_speed = v + w * WHEEL_SEPARATION / 2.0

        fl_cmd = int(fl_speed * VELOCITY_SCALE)
        fr_cmd = int(fr_speed * VELOCITY_SCALE)
        rl_cmd = int(rl_speed * VELOCITY_SCALE)
        rr_cmd = int(rr_speed * VELOCITY_SCALE)

        self.send_velocity_to_motors(fl_cmd, rl_cmd, fr_cmd, rr_cmd)

    def send_velocity_to_motors(self, fl_val, rl_val, fr_val, rr_val):
        self.groupSyncWrite.clearParam()

        fl = int(fl_val * LEFT_DIR)
        rl = int(-rl_val * LEFT_DIR)
        fr = int(fr_val * RIGHT_DIR)
        rr = int(-rr_val * RIGHT_DIR)

        def to_param(val):
            if val < 0: val += 4294967296
            return [
                DXL_LOBYTE(DXL_LOWORD(val)), DXL_HIBYTE(DXL_LOWORD(val)),
                DXL_LOBYTE(DXL_HIWORD(val)), DXL_HIBYTE(DXL_HIWORD(val))
            ]

        self.groupSyncWrite.addParam(LEFT_FRONT_ID,  to_param(fl))
        self.groupSyncWrite.addParam(LEFT_REAR_ID,   to_param(rl))
        self.groupSyncWrite.addParam(RIGHT_FRONT_ID, to_param(fr))
        self.groupSyncWrite.addParam(RIGHT_REAR_ID,  to_param(rr))

        self.groupSyncWrite.txPacket()

    def stop_motors(self):
        self.send_velocity_to_motors(0, 0, 0, 0)
        for dxl_id in [LEFT_FRONT_ID, LEFT_REAR_ID, RIGHT_FRONT_ID, RIGHT_REAR_ID]:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
        self.get_logger().info("모터 정지 및 토크 해제")

def main(args=None):
    rclpy.init(args=args)
    node = WheelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_motors()
    finally:
        node.portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()