#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher() : Node("number_publisher"), number_(5)
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                         std::bind(&NumberPublisher::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Number got published.");

    }

private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        publisher_->publish(msg);
    }
    int number_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}