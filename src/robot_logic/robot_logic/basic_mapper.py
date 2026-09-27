import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class BasicMapper(Node):
    def __init__(self):
        super().__init__('basic_mapper')

        # Map configuration
        self.resolution = 0.05  # 5 cm per cell
        self.width = 400  # 20 m
        self.height = 400  # 20 m

        self.origin_x = -10.0
        self.origin_y = -10.0

        # Occupancy values:
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

        robot_cell = self.world_to_grid(
            self.robot_x,
            self.robot_y,
        )

        if robot_cell is None:
            return

        angle = scan.angle_min

        for distance in scan.ranges:
            world_angle = self.robot_yaw + angle

            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                # If an obstacle was hit, set cells up to the obstacle as free and the obstacle cell as occupied
                hit_x = self.robot_x + distance * math.cos(world_angle)
                hit_y = self.robot_y + distance * math.sin(world_angle)

                hit_cell = self.world_to_grid(hit_x, hit_y)

                if hit_cell is not None:
                    self.trace_ray(robot_cell, hit_cell, mark_endpoint_occupied=True)

            elif math.isinf(distance):
                # In case distance is infinite (no obstacle detected), set all cells up until max range to empty
                end_x = self.robot_x + scan.range_max * math.cos(world_angle)
                end_y = self.robot_y + scan.range_max * math.sin(world_angle)

                end_cell = self.world_to_grid(end_x, end_y)

                if end_cell is not None:
                    self.trace_ray(robot_cell, end_cell, mark_endpoint_occupied=False)

            angle += scan.angle_increment

        self.publish_map(scan)

    def world_to_grid(self, x: float, y: float):
        grid_x = int((x - self.origin_x) / self.resolution)

        grid_y = int((y - self.origin_y) / self.resolution)

        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return grid_x, grid_y

        return None

    def trace_ray(self, start, end, mark_endpoint_occupied: bool) -> None:
        x0, y0 = start
        x1, y1 = end

        cells = self.bresenham(
            x0,
            y0,
            x1,
            y1,
        )

        if not cells:
            return

        if mark_endpoint_occupied:
            free_cells = cells[:-1]
        else:
            free_cells = cells

        for x, y in free_cells:
            self.set_cell(x, y, 0)

        if mark_endpoint_occupied:
            end_x, end_y = cells[-1]
            self.set_cell(end_x, end_y, 100)

    def set_cell(self, x: int, y: int, value: int):
        if 0 <= x < self.width and 0 <= y < self.height:
            index = y * self.width + x

            # Don't overwrite an occupied cell with free space
            if value == 0 and self.grid[index] == 100:
                return

            self.grid[index] = value

    def bresenham(
        self,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
    ):
        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        x = x0
        y = y0

        while True:
            cells.append((x, y))

            if x == x1 and y == y1:
                break

            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x += sx

            if e2 < dx:
                err += dx
                y += sy

        return cells

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
