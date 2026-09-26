import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


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

        self.timer = self.create_timer(
            0.1,
            self.publish_command,
        )

        self.get_logger().info(f'Publishing velocity commands to {cmd_vel_topic}')

    def publish_command(self):

        linear_speed = (
            self.get_parameter('linear_speed').get_parameter_value().double_value
        )

        angular_speed = (
            self.get_parameter('angular_speed').get_parameter_value().double_value
        )

        message = TwistStamped()

        message.header.stamp = self.get_clock().now().to_msg()

        message.twist.linear.x = linear_speed
        message.twist.angular.z = angular_speed

        self.cmd_vel_publisher.publish(message)

    def stop_bot(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)


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
