#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial
import random
import math
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CacheTurtle
from turtlesim.srv import Kill

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.declare_parameter("turtle_name_prefix", "turtle_")
        self.declare_parameter("spawn_frequency", 1.0)

        self.turtle_name_prefix_ = self.get_parameter("turtle_name_prefix").value
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        self.turtle_counter = 0
        self.alive_turtles_ = []
        self.alive_turtle_publisher_ = self.create_publisher(
            TurtleArray, "alive_turtles", 10
        )
        self.spawn_client_ = self.create_client(
            Spawn, "/spawn")
        
        self.spawn_timer = self.create_timer(
            1/self.spawn_frequency_, self.spawn_new_turtle
        )

        self.kill_client_ = self.create_client(Kill, "/kill")
        
        self.cache_turtle_service = self.create_service(
            CacheTurtle, "cache_turtle", self.callback_cache_turtle
        )


    def callback_cache_turtle(
            self, request: CacheTurtle.Request, response: CacheTurtle.Response):
        """
        request: be_killed_turtle_name
        """
        kill_name = request.name
        self.call_kill_service(turtle_name=kill_name)
        response.success = True
        return response

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtle_publisher_.publish(msg)


    def spawn_new_turtle(self):
        self.turtle_counter += 1
        name = self.turtle_name_prefix_ + str(self.turtle_counter)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2*math.pi)
        self.call_spawn_service(name, x, y, theta)

    def call_spawn_service(self, turtle_name, x, y, theta):
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for spawn service...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = self.spawn_client_.call_async(request)
        
        future.add_done_callback(
            partial(self.callback_call_spawn_service, request=request))

    def callback_call_spawn_service(self, future, request: Spawn.Request):
        response: Spawn.Response = future.result()
        if response.name != "":
            self.get_logger().info("New alive turtle: " + response.name)
            new_turtle = Turtle()
            new_turtle.name = response.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            self.alive_turtles_.append(new_turtle)
            self.publish_alive_turtles()

    def call_kill_service(self, turtle_name):
        while not self.kill_client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for kill service...")
        
        request = Kill.Request()
        request.name = turtle_name
        future = self.kill_client_.call_async(request)

        future.add_done_callback(
            partial(self.callback_call_kill_service, name=request.name))

    def callback_call_kill_service(self, future, name):
        for (i, turtle) in enumerate(self.alive_turtles_):
            if turtle.name == name:
                del self.alive_turtles_[i]
                self.publish_alive_turtles()
                break

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()