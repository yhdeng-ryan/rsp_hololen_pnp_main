#include <main/hololens_sub.hpp>


hololensSubscriber::hololensSubscriber(const std::string &topic)
    : Node(topic)
{
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        topic,
        10,
        std::bind(&hololensSubscriber::messageCallback, this, std::placeholders::_1));
    bool new_data = false;
}

void hololensSubscriber::getPose(geometry_msgs::msg::Pose &ret_pose)
{
    ret_pose = Pose;
    new_data = false;
}

bool hololensSubscriber::isMessageReady()
{
    return new_data;
}

void hololensSubscriber::messageCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    new_data = true;
    Pose.position = msg->position;
    RCLCPP_INFO_STREAM(get_logger(), "New incoming message from HoloLens");
    // std::cout << "Received pose message: (" << msg->position.x << ", "
    //           << msg->position.y << ", " << msg->position.z << ")" << std::endl;
}
