#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

class dummyHoloLensRegPub : public rclcpp::Node
{
public:
  dummyHoloLensRegPub()
    : Node("registration_pose_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("marker_pose", 10);

    std::thread keyboard_thread(&dummyHoloLensRegPub::keyboardInputLoop, this);
    keyboard_thread.detach();
  }

private:
  void keyboardInputLoop()
  {
    while (rclcpp::ok())
    {
      std::cout << "Enter reg x, y, and z coordinates separated by spaces: ";
      double x, y, z;
      std::cin >> x >> y >> z;

      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;

      RCLCPP_INFO(this->get_logger(), "Publishing pose: (%f, %f, %f)", x, y, z);
      publisher_->publish(pose);

      std::this_thread::sleep_for(1s);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dummyHoloLensRegPub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

