import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import random

class ExplorerNode(Node):
    def __init__(self):
        super().__init__("explorer_node")
        self.map_subscription = self.create_subscription(OccupancyGrid, "map", self.map_callback,10)
        self.action_client = ActionClient(self,FollowWaypoints,"follow_waypoints")
        self.action_client.wait_for_server()
        self.robot_is_driving = False
        self.create_timer(1.0,self.control_loop)
        self.latest_map = None

    '''This function runs in the background whenever SLAM updates the map. 
       We simply save the data so we can check it later.'''

    def map_callback(self,msg):
        self.latest_map =msg
    '''It converts World Coordinates (meters) into Map Pixels (indices) to ensure we don't crash into a wall.'''

        
    def get_random_unexplored_cell(self):
        attempts = 0

        while attempts < 1000:
            attempts +=1
            #pick random floats
            random_x = random.uniform(-5,5)
            random_y = random.uniform(-5,5)
            # 2. MATH: Convert Meters -> Map Grid Index
            # (Subtract origin offset, divide by resolution)
            local_x = random_x - self.latest_map.info.origin.position.x
            local_y = random_y - self.latest_map.info.origin.position.y
            # Convert to grid indices
            grid_x = int(local_x / self.latest_map.info.resolution)
            grid_y = int(local_y / self.latest_map.info.resolution)
            # Calculate 1D index
            width = self.latest_map.info.width
            height = self.latest_map.info.height
            index = (grid_y * width) + grid_x
            #check bounds
            if index < 0 or index >= len(self.latest_map.data):
                continue
            #check safety
            if self.latest_map.data[index] != 0:
               continue
            
            # We check Up, Down, Left, Right indices
            neighbors = [
                index + 1, #right
                index - 1, #left
                index + width, #up
                index - width #down
            ]

            is_frontier = False
            for n in neighbors:
                # Ensure neighbor is inside array bounds
                if 0<=n < len(self.latest_map.data):
                    # If any neighbor is Unknown (-1), this is a Frontier!
                    if self.latest_map.data[n] == -1:
                        is_frontier = True
                        break
            if is_frontier:
                self.get_logger().info(f"Found frontier at ({random_x}, {random_y})")
                return random_x, random_y
        
        self.get_logger().info("No frontiers found")
        return random_x, random_y



    def control_loop(self):
        if self.latest_map is None:
            return
        if self.robot_is_driving:
            return
        x,y = self.get_random_unexplored_cell()
        self.get_logger().info(f"Moving to random cell: ({x}, {y})")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        
        # 3. Build the Action Goal
        # FollowWaypoints expects a LIST of poses
        msg = FollowWaypoints.Goal()
        #waypoint are always posed as in a list of poses
        msg.poses = [goal_pose]

        self.robot_is_driving = True
        self.future = self.action_client.send_goal_async(msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("goal was rejected")
            self.robot_is_driving = False
            return
        self.get_logger().info("goal accepted")
        self.future = goal_handle.get_result_async()
        self.future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self,future):
        result = future.result().result
        self.get_logger().info("Arrived at destination!") 
        # The robot has arrived! 
        # Reset the flag so the timer can pick a NEW point.
        self.robot_is_driving = False


def main():
    rclpy.init()
    explorer_node = ExplorerNode()
    rclpy.spin(explorer_node)
    explorer_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    


            
            
        

