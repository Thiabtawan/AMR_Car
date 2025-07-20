#!/usr/bin/env python3
import time
import math
import serial
import threading

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

def quaternion_from_yaw(yaw: float):
    """
    คำนวณ quaternion จาก yaw (roll=pitch=0)
    คืนค่า list [qx, qy, qz, qw]
    """
    half = yaw * 0.5
    qw = math.cos(half)
    qz = math.sin(half)
    return [0.0, 0.0, qz, qw]

class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_to_odom')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('ticks_per_rev', 1024)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_base', 0.15)
        self.declare_parameter('poll_rate', 20.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        self.TPR = self.get_parameter('ticks_per_rev').value
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value
        rate = self.get_parameter('poll_rate').value

        # Open serial port and clear buffers
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.get_logger().info(f"Serial open on {port} @ {baud}bps")

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_enc_l = 0
        self.last_enc_r = 0
        self.lock = threading.Lock()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_br = TransformBroadcaster(self)

        # Timer for polling encoder data
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def timer_callback(self):
        with self.lock:
            # Send read command and read all available bytes
            self.ser.reset_input_buffer()
            self.ser.write(b"e\r\n")
            time.sleep(0.05)
            raw = self.ser.read_all()
            line = raw.decode(errors='ignore').strip()
            if not line:
                self.get_logger().warn(f"Empty reply (raw={raw!r})")
                return

        try:
            enc_l, enc_r = [int(i) for i in line.split()]
        except ValueError:
            self.get_logger().warn(f"Invalid reply from Arduino: '{line}'")
            return
        self.get_logger().info(f"Read encoders: left={enc_l}, right={enc_r}")
        # Compute distance moved
        delta_l = enc_l - self.last_enc_l
        delta_r = enc_r - self.last_enc_r
        self.last_enc_l = enc_l
        self.last_enc_r = enc_r

        dist_l = (2 * math.pi * self.R) * (delta_l / self.TPR)
        dist_r = (2 * math.pi * self.R) * (delta_r / self.TPR)

        # Update pose
        delta_s = (dist_r + dist_l) / 2.0
        delta_th = (dist_r - dist_l) / self.L
        dx = delta_s * math.cos(self.th + delta_th / 2.0)
        dy = delta_s * math.sin(self.th + delta_th / 2.0)
        self.x += dx
        self.y += dy
        self.th += delta_th

        # Create quaternion
        q = quaternion_from_yaw(self.th)

        # Publish Odometry message
        odom = Odometry()
        now = self.get_clock().now().to_msg()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.odom_pub.publish(odom)

        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_br.sendTransform(t)

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
