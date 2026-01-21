import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarReader(Node):
    def __init__(self):
        super().__init__("lidar_reader")
        self.subscription = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.get_logger().info("Lidar reader initialized")

    def scan_callback(self,msg):
        #helper to get distnce at specific angle
        def get_distance(angle):
            # Calculate index logic
            if angle < msg.angle_min:
                angle = msg.angle_min
            if angle > msg.angle_max:
                angle = msg.angle_max
            index = int((angle - msg.angle_min)/ msg.angle_increment)
            # <--- SAFETY CLAMP ADDED HERE
            if index >= len(msg.ranges):
                index = len(msg.ranges) - 1
            return msg.ranges[index]

        front = get_distance (0.0)
        left = get_distance(math.pi/2)
        right = get_distance(-math.pi/2)
        back = get_distance(-math.pi)
        self.get_logger().info(f"Front: {front}, Left: {left}, Right: {right}, Back: {back}")
    


def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LidarReader()
    rclpy.spin(lidar_reader)
    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()