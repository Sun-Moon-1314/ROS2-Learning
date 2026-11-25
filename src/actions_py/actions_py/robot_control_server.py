#!/usr/bin/env python3
import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import RobotControl
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class RobotControlServerNode(Node):
    def __init__(self):
        super().__init__("robot_control_server")
        self.robot_position_ = 50

        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()

        self.robot_control_server_ = ActionServer(
            self, 
            RobotControl, 
            "robot_control",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("Robot control action server has been started.")
        self.get_logger().info("robot_position: " + str(self.robot_position_))
    

    def goal_callback(self, goal_request: RobotControl.Goal):
        self.get_logger().info("Received a goal.")
 
        target_position = goal_request.target_position
        velocity = goal_request.velocity

        if target_position not in range(0, 100) or velocity <= 0:
            self.get_logger().warn("Rejecting the goal, Invalid information.")
            return GoalResponse.REJECT

        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and receive new goal.")
                self.goal_handle_.abort()

        self.get_logger().info("Accept the goal.")

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Receive a cancel request.")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        
        target_position = goal_handle.request.target_position
        velocity = goal_handle.request.velocity

        result = RobotControl.Result()
        feedback = RobotControl.Feedback()
        self.get_logger().info("Execute the goal.")

        while rclpy.ok():

            if not goal_handle.is_active:
                result.real_position = self.robot_position_
                result.message = "Preempted by another goal."
                return result

            if goal_handle.is_cancel_requested:
                result.real_position = self.robot_position_
                if self.robot_position_ == target_position:
                    result.message = "Success"
                    goal_handle.succeed()
                else:
                    result.message = "Canceled"
                    goal_handle.canceled()
                return result
            
            diff = target_position - self.robot_position_

            if diff == 0:
                result.real_position = self.robot_position_
                result.message = "Success"
                goal_handle.succeed()
                return result 
            elif diff > 0:
                if diff >= velocity:
                    self.robot_position_ += velocity
                else:
                    self.robot_position_ += diff
            else:
                if abs(diff) >= velocity:
                    self.robot_position_ -= velocity
                else:
                    self.robot_position_ += diff

            self.get_logger().info("Robot position: " + str(self.robot_position_))

            feedback.current_position = self.robot_position_
            goal_handle.publish_feedback(feedback)

            time.sleep(1.0)

        return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()