#include <robotiq_2f_msgs/msg/command_state.hpp>
#include <rclcpp/rclcpp.hpp>

class gripper_publisher : public rclcpp::Node
{
    private:
    rclcpp::Publisher<robotiq_2f_msgs::msg::CommandState>::SharedPtr gripper_pub;

    public:
    gripper_publisher(const std::string &name);
    void publish(std::string command);
};