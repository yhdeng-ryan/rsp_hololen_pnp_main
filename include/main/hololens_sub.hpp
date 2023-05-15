#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

class hololensSubscriber : public rclcpp::Node
{
public:
    hololensSubscriber(const std::string &topic);

    void getPose(geometry_msgs::msg::Pose& ret_pose);

    bool isMessageReady();
    

private:
    void messageCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    bool new_data;
    geometry_msgs::msg::Pose Pose;
};