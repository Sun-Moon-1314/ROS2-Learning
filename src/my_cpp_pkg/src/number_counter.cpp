#include <functional>
#include <memory>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode()
        : Node("number_counter"),
          counter_(0)
    {
        // 创建发布者
        counter_publisher_ =
            this->create_publisher<example_interfaces::msg::Int64>(
                "counter", 10);

        // 创建订阅者
        number_subscriber_ =
            this->create_subscription<example_interfaces::msg::Int64>(
                "number",
                10,
                std::bind(
                    &NumberCounterNode::callbackNumber,
                    this,
                    _1));

        // 创建服务
        reset_counter_service_ =
            this->create_service<example_interfaces::srv::SetBool>(
                "reset_counter",
                std::bind(
                    &NumberCounterNode::callbackResetCounter,
                    this,
                    _1,
                    _2));

        RCLCPP_INFO(
            this->get_logger(),
            "Number Counter has been started.");
    }

private:
    // 订阅者回调函数
    void callbackNumber(
        const example_interfaces::msg::Int64::SharedPtr msg)
    {
        // 累加收到的数字
        counter_ += msg->data;

        // 创建新的消息
        example_interfaces::msg::Int64 new_msg;
        new_msg.data = counter_;

        // 发布当前计数值
        counter_publisher_->publish(new_msg);
    }

    // 服务回调函数
    void callbackResetCounter(
        const example_interfaces::srv::SetBool::Request::SharedPtr request,
        example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data)
        {
            counter_ = 0;
            response->success = true;
            response->message = "Counter has been reset.";
        }
        else
        {
            response->success = false;
            response->message = "Counter has not been reset.";
        }

        // C++不需要 return response
        // 因为 response 是指针，已经直接修改了它的内容
    }

    int64_t counter_;

    rclcpp::Publisher<
        example_interfaces::msg::Int64
    >::SharedPtr counter_publisher_;

    rclcpp::Subscription<
        example_interfaces::msg::Int64
    >::SharedPtr number_subscriber_;

    rclcpp::Service<
        example_interfaces::srv::SetBool
    >::SharedPtr reset_counter_service_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NumberCounterNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}