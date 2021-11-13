#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
class NumberCounter : public rclcpp::Node
{
public:
    NumberCounter() : Node("number_counter"), counter_(0)
    {
        counter_publisher=this->create_publisher<example_interfaces::msg::Int64>("number_count",10);
        
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10,
            std::bind(&NumberCounter::callbackNews, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "number_counter started. ");
    }


private:
    void callbackNews(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        auto new_msg= example_interfaces::msg::Int64();
        new_msg.data=counter_;
        counter_publisher->publish(new_msg);

        
    }
    int counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr counter_publisher;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}