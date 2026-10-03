import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class DriveDemo(Node):
    def __init__(self):
        super().__init__('drive_demo')

        self.declare_parameter(
            'cmd_vel_topic',
            '/diff_drive_controller/cmd_vel',
        )

        self.declare_parameter(
            'linear_speed',
            0.5,
        )

        self.declare_parameter(
            'angular_speed',
            0.35,
        )

        cmd_vel_topic = (
            self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        )

        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            cmd_vel_topic,
            10,
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        self.get_logger().info(f'Publishing velocity commands to {cmd_vel_topic}')

    def stop_bot(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)

    def obstacle_ahead(self, scan: LaserScan, max_distance=1.0, half_angle_deg=30.0):
        half_angle = math.radians(half_angle_deg)

        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment

            if -half_angle <= angle <= half_angle and (
                math.isfinite(distance)
                and scan.range_min <= distance <= scan.range_max
                and distance < max_distance
            ):
                return True

        return False

    def scan_callback(self, msg: LaserScan):
        if self.obstacle_ahead(msg):
            angular_speed = (
                self.get_parameter('angular_speed').get_parameter_value().double_value
            )

            message = TwistStamped()

            message.header.stamp = self.get_clock().now().to_msg()

            message.twist.linear.x = 0.0
            message.twist.angular.z = angular_speed

            self.cmd_vel_publisher.publish(message)
        else:
            linear_speed = (
                self.get_parameter('linear_speed').get_parameter_value().double_value
            )

            message = TwistStamped()

            message.header.stamp = self.get_clock().now().to_msg()

            message.twist.linear.x = linear_speed
            message.twist.angular.z = 0.0

            self.cmd_vel_publisher.publish(message)


def main(args=None):

    rclpy.init(args=args)

    node = DriveDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_bot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
