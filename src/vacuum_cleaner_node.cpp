#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>


#include <geometry_msgs/msg/twist.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class TurtleBotNode : public rclcpp::Node
{
public:
  TurtleBotNode()
  : Node("turtlebot3_main_cpp")
  {
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::ServicesQoS());

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TurtleBotNode::publish_cmd_vel_function, this));
  
  } 

  void publish_cmd_vel_function()
  {
      auto twist_msg = geometry_msgs::msg::Twist();

      twist_msg.linear.x = 0.0;
      twist_msg.linear.y = 0.0;


      twist_msg.angular.z = 0.0;

      RCLCPP_INFO(this->get_logger(), "Publishing: 1");
      cmd_vel_publisher_->publish(twist_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBotNode>());
  rclcpp::shutdown();
  return 0;
}