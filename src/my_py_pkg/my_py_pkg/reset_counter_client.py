#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from functools import partial

class ResetCounterClient(Node): # modify name
    def __init__(self):
        super().__init__("reset_counter_client") # modify name
        self.client_ = self.create_client(SetBool, "reset_counter")

    def call_reset_counter(self, data: bool):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for Reset Counter server...")
        
        request = SetBool.Request()
        request.data = data

        return self.client_.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterClient()
    future = node.call_reset_counter(data=True)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    node.get_logger().info(
        f"success={response.success}, "
        f"message={response.message}"
    )
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()