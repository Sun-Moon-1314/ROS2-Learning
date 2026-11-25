#!/usr/bin/env python3
import math
import rclpy
from functools import partial
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CacheTurtle
 

class TurtleControlNode(Node):
    def __init__(self):
        super().__init__("turtle_control")
        self.declare_parameter("catch_closest_turtle_first", True)
        self.catch_closest_turtle_first_ = self.get_parameter(
            "catch_closest_turtle_first").value

        self.turtle_to_cache_: Turtle = None
        
        self.pose_ = None
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10
        ) # get turtle current pose
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10
        ) # get prepared turtle lists including some info about target state
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )
        self.cache_turtle_client_ = self.create_client(
            CacheTurtle, "cache_turtle"
        )
        self.control_loop_timer = self.create_timer(
            0.01, self.control_loop
        )

    def callback_alive_turtles(self, msg: TurtleArray):
        turtles = msg.turtles
        if len(turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None
                for turtle in turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                    
                self.turtle_to_cache_ = closest_turtle
            else:
                self.turtle_to_cache_ = msg.turtles[0]

    def callback_pose(self, pose: Pose):
        self.pose_ = pose

    def control_loop(self):
        if self.pose_ == None or self.turtle_to_cache_ == None:
            return

        cmd = Twist()
        
        dist_x = self.turtle_to_cache_.x - self.pose_.x
        dist_y = self.turtle_to_cache_.y - self.pose_.y

        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        if distance > 0.5:
            cmd.linear.x = 2 * distance
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            
            cmd.angular.z = 6 * diff
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_cache_.name)
            self.turtle_to_cache_ = None

        self.cmd_vel_publisher_.publish(cmd)

    def call_catch_turtle_service(self, turtle_name):
        while not self.cache_turtle_client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for cache turtle service...")
                
        request = CacheTurtle.Request()
        request.name = turtle_name
        future = self.cache_turtle_client_.call_async(request)

        future.add_done_callback(
            partial(self.callback_call_catch_turtle_service, 
                        name=request.name))

    def callback_call_catch_turtle_service(self, future, name):
        response: CacheTurtle.Response = future.result()
        if not response.success:
            self.get_logger().info(
                "Turtle " + name + " could not be removed.")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()