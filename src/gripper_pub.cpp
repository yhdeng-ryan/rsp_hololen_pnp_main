#include <main/gripper_pub.hpp>

gripper_publisher::gripper_publisher(const std::string &name): Node(name)
{
    gripper_pub = create_publisher<robotiq_2f_msgs::msg::CommandState>("robotiq_2f_command", 10);
}

void gripper_publisher::publish(std::string command)
{
    auto cmd = robotiq_2f_msgs::msg::CommandState();
    cmd.command = command;
    gripper_pub -> publish(cmd);
}