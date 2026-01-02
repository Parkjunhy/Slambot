#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import *

# ==============================
# 1. 다이나믹셀 / 로봇 설정
# ==============================
LEFT_FRONT_ID  = 3
LEFT_REAR_ID   = 2
RIGHT_FRONT_ID = 4
RIGHT_REAR_ID  = 1

DEVICENAME       = '/dev/ttyACM0'
BAUDRATE         = 115200
PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE  = 64
ADDR_GOAL_VELOCITY  = 104  # 4 byte

VELOCITY_CONTROL_MODE = 1  # 바퀴 모드

# 기구학 파라미터
WHEEL_SEPARATION = 0.15    # [m]
VELOCITY_SCALE   = 500     # [m/s] → 모터 단위

# 좌/우 방향 보정 (오른쪽 모터가 뒤집혀 있으면 -1)
LEFT_DIR  = 1
RIGHT_DIR = -1

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            ADDR_GOAL_VELOCITY,
            4
        )

        if not self.portHandler.openPort():
            self.get_logger().error("포트를 열 수 없습니다.")
            sys.exit(1)

        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("BAUDRATE 설정 실패!")
            sys.exit(1)

        self.setup_motors()

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("WheelController 준비 완료. /cmd_vel 기다리는 중...")

    def setup_motors(self):
        # 토크 OFF
        for dxl_id in [LEFT_FRONT_ID, LEFT_REAR_ID, RIGHT_FRONT_ID, RIGHT_REAR_ID]:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0
            )

        # 속도 제어 모드
        for dxl_id in [LEFT_FRONT_ID, LEFT_REAR_ID, RIGHT_FRONT_ID, RIGHT_REAR_ID]:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE
            )

        # 토크 ON
        for dxl_id in [LEFT_FRONT_ID, LEFT_REAR_ID, RIGHT_FRONT_ID, RIGHT_REAR_ID]:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1
            )

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x      # [m/s]
        w = msg.angular.z     # [rad/s]

        self.get_logger().info(
            f"/cmd_vel → v={v:.3f}, w={w:.3f}"
        )

        # --------------------------
        # 앞/뒤 바퀴를 다르게 처리
        # --------------------------
        # 앞바퀴: FL = v - wL/2, FR = v + wL/2
        # 뒷바퀴: RL = v + wL/2, RR = v - wL/2 (회전 성분만 반대로)
        fl_speed = v - w * WHEEL_SEPARATION / 2.0
        fr_speed = v + w * WHEEL_SEPARATION / 2.0
        rl_speed = v + w * WHEEL_SEPARATION / 2.0
        rr_speed = v - w * WHEEL_SEPARATION / 2.0

        fl_cmd = int(fl_speed * VELOCITY_SCALE)
        fr_cmd = int(fr_speed * VELOCITY_SCALE)
        rl_cmd = int(rl_speed * VELOCITY_SCALE)
        rr_cmd = int(rr_speed * VELOCITY_SCALE)

        self.send_velocity_to_motors(fl_cmd, rl_cmd, fr_cmd, rr_cmd)

    def send_velocity_to_motors(self, fl_val, rl_val, fr_val, rr_val):
        self.groupSyncWrite.clearParam()

        # 좌측 (LEFT_DIR)
        fl = fl_val * LEFT_DIR
        rl = -rl_val * LEFT_DIR

        # 우측 (RIGHT_DIR)
        fr = fr_val * RIGHT_DIR
        rr = -rr_val * RIGHT_DIR

        # 유틸 함수
        def to_param(val: int):
            return [
                DXL_LOBYTE(DXL_LOWORD(val)),
                DXL_HIBYTE(DXL_LOWORD(val)),
                DXL_LOBYTE(DXL_HIWORD(val)),
                DXL_HIBYTE(DXL_HIWORD(val)),
            ]

        self.groupSyncWrite.addParam(LEFT_FRONT_ID,  to_param(fl))
        self.groupSyncWrite.addParam(LEFT_REAR_ID,   to_param(rl))
        self.groupSyncWrite.addParam(RIGHT_FRONT_ID, to_param(fr))
        self.groupSyncWrite.addParam(RIGHT_REAR_ID,  to_param(rr))

        # 디버그용 출력
        self.get_logger().info(
            f"CMD FL={fl}, RL={rl}, FR={fr}, RR={rr}"
        )

        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().warn(f"GroupSyncWrite 실패: {dxl_comm_result}")

    def stop_motors(self):
        self.send_velocity_to_motors(0, 0, 0, 0)
        for dxl_id in [LEFT_FRONT_ID, LEFT_REAR_ID, RIGHT_FRONT_ID, RIGHT_REAR_ID]:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0
            )
        self.get_logger().info("모든 모터 정지 + 토크 OFF")

def main(args=None):
    rclpy.init(args=args)
    node = WheelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: 종료 중...")
        node.stop_motors()
    finally:
        node.portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()