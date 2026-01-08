#!/usr/bin/env python3

import os
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR
)
from adafruit_bno08x.i2c import BNO08X_I2C

class BNO08XImuNode(Node):
    def __init__(self):
        super().__init__('bno08x_imu_node')

        try:
            os.system('sudo chmod 666 /dev/i2c-1')
        except Exception as e:
            self.get_logger().error(f"Failed to set I2C permissions: {e}")

        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        
        self.bno = None
        self.setup_sensor()

        self.timer = self.create_timer(0.01, self.publish_imu)
        self.get_logger().info("BNO08X IMU ROS2 node started and stabilized")

    def setup_sensor(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(i2c, address=0x4B)
            
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        except Exception as e:
            self.get_logger().error(f"CRITICAL: Could not find BNO08X sensor: {e}")

    def publish_imu(self):
        if self.bno is None:
            return

        try:
            accel = self.bno.acceleration
            gyro = self.bno.gyro
            quat = self.bno.quaternion
            
            if None in [accel, gyro, quat]:
                return

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = accel

            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyro

            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = quat

            imu_msg.orientation_covariance[0] = -1.0
            imu_msg.angular_velocity_covariance[0] = -1.0
            imu_msg.linear_acceleration_covariance[0] = -1.0

            self.imu_pub.publish(imu_msg)

        except (KeyError, RuntimeError) as e:
            self.get_logger().debug(f"Sensor report skipped: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in loop: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BNO08XImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node crashed: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()