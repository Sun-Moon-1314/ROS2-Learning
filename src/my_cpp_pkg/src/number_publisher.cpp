#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

class NumberPublisherNode : public rclcpp::Node{

public:
    NumberPublisherNode() : Node("number_publisher")
    {
        this->declare_parameter("number", 3);
        this->declare_parameter("time_period", 1.0);

        number_ = get_parameter("number").as_int();
        double time_period_ = this->get_parameter("time_period").as_double();

        publisher_ = 
        this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(time_period_), 
            std::bind(&NumberPublisherNode::callback_number_publisher, this)
        );
        RCLCPP_INFO(get_logger(), "Number publihser has been started");
    }

private:

    void callback_number_publisher()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        publisher_->publish(msg);
    }
    int number_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}