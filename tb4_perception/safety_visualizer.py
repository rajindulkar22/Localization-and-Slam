import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time # <--- Import Time
import math

class SafetyVisualizer(Node):
    def __init__(self):
        super().__init__('safety_visualizer')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.subscription = self.create_subscription(
            LaserScan, 
            'scan', 
            self.scan_callback, 
            10, 
            callback_group=self.callback_group
        )
        
        self.publisher = self.create_publisher(
            Marker, 
            'safety_marker', 
            10, 
            callback_group=self.callback_group
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('Safety visualizer initialized')

    def scan_callback(self, msg):
        shortest_dist = float('inf')
        shortest_index = -1
        
        for i in range(len(msg.ranges)):
            current_range = msg.ranges[i]
            if math.isnan(current_range) or math.isinf(current_range):
                continue
            if current_range < shortest_dist:
                shortest_dist = current_range
                shortest_index = i
        
        if shortest_index == -1:
            return

        angle = msg.angle_min + shortest_index * msg.angle_increment
        x = shortest_dist * math.cos(angle)
        y = shortest_dist * math.sin(angle)

        msg_point = PointStamped()
        msg_point.header.frame_id = msg.header.frame_id 
        
        # --- THE FIX: TIME ZERO ---
        # We explicitly tell TF: "I don't care about the exact millisecond.
        # Just give me the LATEST position you have."
        msg_point.header.stamp = Time().to_msg() 
        
        msg_point.point.x = x
        msg_point.point.y = y

        try:
            # We can lower the timeout now because Time(0) is instant
            transformed_point = self.tf_buffer.transform(
                msg_point, 
                'odom', 
                timeout=Duration(seconds=0.1)
            )
            self.get_logger().info(f"Odom Point: ({transformed_point.point.x:.2f}, {transformed_point.point.y:.2f})")
        except TransformException as e:
            self.get_logger().info(f"Waiting for transform: {e}")
            return
            
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = transformed_point.point.x
        marker.pose.position.y = transformed_point.point.y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    safety_visualizer = SafetyVisualizer()
    executor = MultiThreadedExecutor()
    executor.add_node(safety_visualizer)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        safety_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()