#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from rcl_interfaces.msg import SetParametersResult


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")

        self.declare_parameter("number", 2)
        self.declare_parameter("timer_period", 1.0)

        self.number_ = self.get_parameter("number").value
        self.timer_period_ = self.get_parameter("timer_period").value

        self.add_on_set_parameters_callback(self.params_callback)

        # /number means there is no namespace before it
        self.number_publisher_ = self.create_publisher(
            Int64, "number", 10
        )

        self.number_timer_ = self.create_timer(
            self.timer_period_,
            self.callback_number
        )

        self.get_logger().info(
            "Number Publisher has been started."
        )

    def params_callback(self, params):
        for param in params:
            if param.name == "number":
                self.number_ = param.value

        result = SetParametersResult()
        result.successful = True
        result.reason = "Parameter updated successfully"
        return result

    def callback_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = NumberPublisherNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()