import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped


class PatrolNode(Node):
    def __init__(self):
        super().__init__("patrol_node")
        self.action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.action_client.wait_for_server()
        self.goal = FollowWaypoints.Goal()
        self.goal.poses = self.get_my_waypoint()
        self.get_logger().info("Sending goal")
        self.future = self.action_client.send_goal_async(self.goal)
        self.future.add_done_callback(self.goal_response_callback)
    
    def get_my_waypoint(self):
        #create a list of waypoints which are PoseStamped messages
        poses_list = []
        wp1 = PoseStamped()
        wp1.header.frame_id = "map"
        wp1.header.stamp = self.get_clock().now().to_msg()
        wp1.pose.position.x =1.5
        wp1.pose.position.y =1.5
        wp1.pose.orientation.w =1.0
        poses_list.append(wp1)

        wp2 = PoseStamped()
        wp2.header.frame_id = "map"
        wp2.header.stamp = self.get_clock().now().to_msg()
        wp2.pose.position.x =3.5
        wp2.pose.position.y =1.5
        wp2.pose.orientation.w =1.0
        poses_list.append(wp2)
        
        return poses_list
        


    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("goal was rejected")
            return
        self.get_logger().info("goal accepted")
        self.future = goal_handle.get_result_async()
        self.future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self,future):
        result = future.result().result
        self.get_logger().info("goal result: " + str(result))
        # RECURSION: Start the process again instead of shutting down
        self.get_logger().info("Starting again")
        self.future = self.action_client.send_goal_async(self.goal)
        self.future.add_done_callback(self.goal_response_callback)


def main():
    rclpy.init()
    patrol_node = PatrolNode()
    rclpy.spin(patrol_node)
    patrol_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()