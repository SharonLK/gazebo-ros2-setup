import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class BasicMapper(Node):
    def __init__(self):
        super().__init__('basic_mapper')

        # Map configuration
        self.resolution = 0.05  # 5 cm / cell
        self.width = 400  # 20 m
        self.height = 400  # 20 m

        self.origin_x = -10.0
        self.origin_y = -10.0

        # -1 = unknown
        #  0 = free
        # 100 = occupied
        self.grid = [-1] * (self.width * self.height)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.have_odom = False

        self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10,
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10,
        )

        self.get_logger().info('Basic mapper started')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Quaternion -> yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)

        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.robot_yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        self.have_odom = True

    def scan_callback(self, scan: LaserScan):
        if not self.have_odom:
            return

        angle = scan.angle_min

        for distance in scan.ranges:
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                world_angle = self.robot_yaw + angle

                hit_x = self.robot_x + distance * math.cos(world_angle)

                hit_y = self.robot_y + distance * math.sin(world_angle)

                self.mark_occupied(hit_x, hit_y)

            angle += scan.angle_increment

        self.publish_map(scan)

    def mark_occupied(self, x: float, y: float):
        grid_x = int((x - self.origin_x) / self.resolution)

        grid_y = int((y - self.origin_y) / self.resolution)

        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            index = grid_y * self.width + grid_x
            self.grid[index] = 100

    def publish_map(self, scan: LaserScan):
        msg = OccupancyGrid()

        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = 'odom'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = BasicMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
