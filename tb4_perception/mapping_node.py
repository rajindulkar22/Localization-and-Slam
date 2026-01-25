import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy
import math


class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.map_resolution = 0.1
        self.map_width = 100
        self.map_height =100
        self.map_origin_x = -5.0
        self.map_origin_y = -5.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.grid_data = [50] * (self.map_width * self.map_height)
        latched_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid,"map",latched_qos)
        self.subscription = self.create_subscription(Odometry,"odom",self.odom_callback,10)
        self.scan_subscription = self.create_subscription(LaserScan,"scan",self.scan_callback,10)
        self.costmap_publisher = self.create_publisher(OccupancyGrid,"global_costmap",10)
        self.create_timer(1.0,self.timer_callback)

    def odom_callback(self,msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (roll,pitch,yaw) = euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.robot_theta = yaw
    
    def bresenham(self, x0, y0, x1, y1): #Bresenham's Line Algorithm
        """Yields coordinates for a straight line between (x0,y0) and (x1,y1)."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                yield (x, y)
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                yield (x, y)
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        yield (x, y)


    def scan_callback(self,msg):
        # 1. Calculate Robot's Grid Position (Start of every ray)
        u0= int((self.robot_x - self.map_origin_x)/self.map_resolution)
        v0 = int((self.robot_y - self.map_origin_y)/self.map_resolution)
        for i in range(0,len(msg.ranges),5):
            r=msg.ranges[i]
            if math.isnan(r) or math.isinf(r) or r>10.0:
                continue
            angle = self.robot_theta + (msg.angle_min + i * msg.angle_increment)
            end_x=self.robot_x + r * math.cos(angle)
            end_y=self.robot_y + r * math.sin(angle)
            u1 = int((end_x - self.map_origin_x)/self.map_resolution)
            v1 = int((end_y - self.map_origin_y)/self.map_resolution)
            pixels = list(self.bresenham(u0, v0, u1, v1))
    
            for j, (u, v) in enumerate(pixels):
                if 0 <= u < self.map_width and 0 <= v < self.map_height:
                    idx = u + v * self.map_width        
                    # If it's the LAST pixel, it's the WALL (Occupied)
                    if j == len(pixels) - 1:
                        self.grid_data[idx] += 20
                        if self.grid_data[idx] > 100:
                            self.grid_data[idx] = 100
                    # If it's not the last pixel, it's AIR (Free)
                    else:
                        self.grid_data[idx] -= 5
                        if self.grid_data[idx] < 0:
                            self.grid_data[idx] = 0

    def timer_callback(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height =self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.data = self.grid_data
        self.publisher.publish(msg)


        costmap = list(self.grid_data)
        inflation_radius = 2
        # 2. Iterate through the ORIGINAL grid
        for i in range(len(self.grid_data)):
            # If the ORIGINAL map has a wall here
            if self.grid_data[i] > 50:
                # Convert 1D index 'i' back to 2D (u, v)
                u = i % self.map_width
                v = i // self.map_width
                # Iterate through the INFLATION RADIUS
                for dx in range(-inflation_radius, inflation_radius +1):
                    for dy in range(-inflation_radius, inflation_radius +1):
                        # Calculate new coordinates
                        new_u = u + dx
                        new_v = v + dy
                        # SAFETY CHECK: Stay inside map boundaries!
                        if 0<=new_u<self.map_width and 0<=new_v<self.map_height:
                            new_idx = new_u + new_v * self.map_width
                            costmap[new_idx] = 100

        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = "map"
        costmap_msg.info.resolution = self.map_resolution
        costmap_msg.info.width = self.map_width
        costmap_msg.info.height =self.map_height
        costmap_msg.info.origin.position.x = self.map_origin_x
        costmap_msg.info.origin.position.y = self.map_origin_y
        costmap_msg.data = costmap
        self.costmap_publisher.publish(costmap_msg)

        
    
def main(args=None):
    rclpy.init(args=args)
    mapping_node = MappingNode()
    rclpy.spin(mapping_node)
    mapping_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



    
