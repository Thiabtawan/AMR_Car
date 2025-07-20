import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        # parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_mode', 'Express')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        scan_mode = self.get_parameter('scan_mode').get_parameter_value().string_value

        # init publisher
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.lidar = RPLidar(port, baudrate=baud)
        self.lidar.set_pwm(660)  # start motor
        self.get_logger().info(f"RPLidar initialized on {port} @ {baud} bps, mode: {scan_mode}")

        # timer for reading scans
        self.timer = self.create_timer(0.1, self.scan_callback)

    def scan_callback(self):
        # read a single scan
        scan = self.lidar.iter_scans(max_buf_meas=2000, scan_mode=self.get_parameter('scan_mode').get_parameter_value().string_value)
        try:
            for (_, angle, distance) in next(scan):
                # construct LaserScan message
                msg = LaserScan()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.angle_min = 0.0
                msg.angle_max = 2.0 * 3.14159265359
                msg.angle_increment = 2.0 * 3.14159265359 / 360.0
                msg.time_increment = 0.0
                msg.scan_time = 1.0 / 10.0
                msg.range_min = 0.15
                msg.range_max = 12.0
                # fill ranges and intensities
                msg.ranges = [float('nan')] * 360
                msg.intensities = [0.0] * 360
                index = int(angle)
                if 0 <= index < 360:
                    msg.ranges[index] = distance / 1000.0
                # publish
                self.publisher.publish(msg)
                break
        except StopIteration:
            self.get_logger().warn('No scan data received')

    def destroy_node(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
