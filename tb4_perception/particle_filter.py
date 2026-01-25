import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Quaternion, Pose
from nav_msgs.msg import Odometry , OccupancyGrid
from sensor_msgs.msg import LaserScan
import random  # for publishing particles
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math

#the odometry message gives you linear speed and angular speed
#we need to use this to update the position of the particles

class Particle():
    def __init__(self,x,y,theta,weight):
        self.x = x
        self.y = y
        self.theta =theta
        self.weight = weight



class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__("particle_filter_node")
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher = self.create_publisher(PoseArray,"particle_cloud",10)
        self.odom_subscription = self.create_subscription(Odometry, "odom", self.odom_callback,10)
        self.map_subscription = self.create_subscription(OccupancyGrid, "map", self.map_callback,10)
        self.laser_subscription = self.create_subscription(LaserScan, "scan", self.laser_callback,10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, 
            "map", 
            self.map_callback, 
            map_qos
        )
        self.map_data = []
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin = [0.0,0.0]
        self.particles = []
        self.num_particles = 1000
        #create a particle cloud
        #loop 1000 times
        for i in range(self.num_particles):
            x= random.uniform(-5,5)
            y = random.uniform(-5,5)
            theta = random.uniform(0,2*math.pi)
            #create a particle object and add it to the list
            p = Particle(x,y,theta,1.0)
            self.particles.append(p) # your particles are initialized at random positions
        self.create_timer(0.1,self.timer_callback)
        self.last_time = self.get_clock().now().nanoseconds / 1e9

    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin[0] = msg.info.origin.position.x
        self.map_origin[1] = msg.info.origin.position.y
        self.get_logger().info("map received")

    def get_simulated_distance(self, x,y , theta):
        if not self.map_data:
            return 10.0
        #convert the start position meter to grid
        #we need to know which pixel c and r the particle is standing at 
        start_c = int((x - self.map_origin[0])/ self.map_resolution)
        start_r = int((y - self.map_origin[1])/ self.map_resolution)
        # The Loop (Walking the Beam)
        max_steps = 100
        for step in range(max_steps):
            #current distance
            distance = step * self.map_resolution
            #New Pixel Coordinates:
            check_c = start_c + int(distance * math.cos(theta)/ self.map_resolution)
            check_r = start_r + int(distance * math.sin(theta)/ self.map_resolution)
            #check if the new pixel is inside the map (bounds checking)
            if check_c < 0 or check_c >= self.map_width or check_r < 0 or check_r >=self.map_height:
                return distance
            #wall check
            index = check_c + check_r * self.map_width
            if self.map_data[index]>50 or self.map_data[index]==100:
                return distance
        return 10.0

    def resample_particles(self):
        weights = [p.weight for p in self.particles]
        max_weight = max(weights)
        average_weight = sum(weights)/self.num_particles
        self.get_logger().info(f"Average Weight: {average_weight:.4f}")


        beta = 0.0
        index = random.randint(0,self.num_particles-1) #random index for the first particle
        new_particles = []
        #create new particles
        for n in range(self.num_particles):
            beta += random.uniform(0,2 * max_weight)
            #resample
            while beta > weights[index]:
                beta -= weights[index]
                index = (index +1) % self.num_particles
            if average_weight < 1.0 and random.random() < 0.10:
                P_x= random.uniform(-5,5)
                P_y = random.uniform(-5,5)
                P_theta = random.uniform(0,2*math.pi)
                #create a particle object and add it to the list
                p = Particle(P_x,P_y,P_theta,1.0)
                new_particles.append(p)  
            else:   
                selected_particle = self.particles[index]

        #add noise to the particles
                new_x = selected_particle.x + random.gauss(0,0.1)
                new_y = selected_particle.y + random.gauss(0,0.1)
                new_theta = selected_particle.theta + random.gauss(0,0.1)

                new_particles.append(Particle(
                    new_x,
                    new_y,
                    new_theta,
                    selected_particle.weight
                ))
        
        self.particles = new_particles


    def laser_callback(self,msg):
        real_distance = msg.ranges[len(msg.ranges)//2] #this takes the middle value of the laser scan
        for p in self.particles:
            simulated_distance = self.get_simulated_distance(p.x,p.y,p.theta)
            error = abs(real_distance - simulated_distance)
            p.weight = 1.0 / (error + 0.1)
        best_particle = max(self.particles, key=lambda p: p.weight)
        self.get_logger().info(f"Best Match Weight: {best_particle.weight:.4f}")
        self.resample_particles()


            
                

    

    def odom_callback(self, msg):
        #calulate the time difference between the current and previous odometry message
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time
        #get velocity from odometry message
        linear_velocity  = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z
        #calculate the movement
        delta_distance = linear_velocity * dt
        delta_theta= angular_velocity * dt

        # skip if the robot is not moving
        if abs(delta_distance) < 0.001 and abs(delta_theta) < 0.001:
            return 

        for p in self.particles:
            #add noise 
            noise_distance = random.gauss(0,0.05 * abs(delta_distance)) + delta_distance
            noise_theta = random.gauss(0,0.05 * abs(delta_theta)) + delta_theta


            #update the position of the particles
            p.x = p.x + noise_distance * math.cos(p.theta)
            p.y = p.y + noise_distance * math.sin(p.theta)
            p.theta = p.theta + noise_theta

        

    def timer_callback(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for p in self.particles:
            pose = Pose()
            pose.position.x = p.x
            pose.position.y = p.y
            quaternion = Quaternion()
            quaternion = quaternion_from_euler(0, 0, p.theta)
            #assign the quaternion values to the pose
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            msg.poses.append(pose)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    particle_filter_node = ParticleFilterNode()
    rclpy.spin(particle_filter_node)
    particle_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

            


    

