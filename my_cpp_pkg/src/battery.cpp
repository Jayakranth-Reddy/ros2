#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node 
{
public:
    BatteryNode() : Node("battery"), battery_state_("full")
    {
        last_time_battery_state_changed_= this->get_clock()->now().seconds();
        battery_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BatteryNode::checkBatterystate, this));
        RCLCPP_INFO(this->get_logger(), "Battery node has been started.");
    }

private:
    void setLed(int led_number, int state)
    {
        threads_.push_back(std::thread(std::bind(&BatteryNode::callSetLedservice, this,led_number, state)));

    }

    void callSetLedservice(int led_number,int state)
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service server to be up...");
        }

        auto request= std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = led_number;
        request->state = state;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "service call failed");
        }


    }
    void checkBatterystate()
    {
        double time_now =this->get_clock()->now().seconds();
        if (battery_state_ == "full")
        {
            if (time_now - last_time_battery_state_changed_ > 4.0)
            {
                RCLCPP_INFO(this->get_logger(), "Battery is now full again.");
                battery_state_ = "full";
                last_time_battery_state_changed_ = time_now;
                setLed(3,0);
            }

        }
        else
        {
            if (time_now - last_time_battery_state_changed_ > 6.0)
            {
                RCLCPP_INFO(this->get_logger(), "Battery is now full again.");
                battery_state_ = "full";
                last_time_battery_state_changed_ = time_now;
                setLed(3,0);
            }
        }

    }




    std::string battery_state_;
    double last_time_battery_state_changed_;

    rclcpp::TimerBase::SharedPtr battery_timer_;
    std::vector<std::shared_ptr<std::thread>> set_led_threads_;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}