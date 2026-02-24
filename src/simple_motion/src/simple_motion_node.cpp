#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SimpleMotionNode : public rclcpp::Node
{
public:
    SimpleMotionNode()
    : Node("simple_motion_node")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual", 10);

        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/simple_motion_command", 10,
            std::bind(&SimpleMotionNode::commandCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Simple Motion Node Started");
    }

private:
    void commandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received command: %s", msg->data.c_str());

        geometry_msgs::msg::Twist cmd;

        if (msg->data == "forward")
            cmd.linear.x = 0.5;
        else if (msg->data == "backward")
            cmd.linear.x = -0.5;
        else if (msg->data == "left")
            cmd.angular.z = 0.5;
        else if (msg->data == "right")
            cmd.angular.z = -0.5;
        else if (msg->data == "stop")
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown command");
            return;
        }

        cmd_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Published cmd_vel_manual");
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMotionNode>());
    rclcpp::shutdown();
    return 0;
}