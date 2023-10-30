#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vacuum_cleaner_pkg/msg/bumper.hpp"



using std::placeholders::_1;

class TurtleBotControl : public rclcpp::Node
{
public:
  TurtleBotControl()
  : Node("turtlebot3_main_cpp")
  {
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::ServicesQoS());

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, std::bind(&TurtleBotControl::odomCallback, this, _1));

    bumper_subscription_ = this->create_subscription<vacuum_cleaner_pkg::msg::Bumper>(
      "/bumper_sensor", 20, std::bind(&TurtleBotControl::bumperCallback, this, _1));

    timer_main = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&TurtleBotControl::main, this));

  } 


  void main()
  {
    random_walk();
  }

  void random_walk()
  {
    if (act_val_bumper && (!collison_handling))
    {
      generate_new_angle_goal();
      collison_handling = true;
    }
    else if (collison_handling)
    {
      collison_handling = collisionHandle();
    }
    else if ((!act_val_bumper) && (!collison_handling))
    {
      publish_cmd_vel_function(1.0, 0.0, 0.0);
    }   
  }


  void generate_new_angle_goal()
  {
    // TO DO act yaw not in range
    angle_goal_val =  get_random_value_from_range(-3.14, 3.14);
    RCLCPP_INFO(this->get_logger(), "Publishing random float: '%f'", angle_goal_val);
  }

  bool collisionHandle()
  {
    float z_vel = (angle_goal_val - act_val_yaw) * K_ANGULAR;
    if (z_vel < 0.1)
    {
      publish_cmd_vel_function(0.0, 0.0, 0.0);
      return false;
    }
    else
    {
      publish_cmd_vel_function(0.0, 0.0, z_vel);
      return true;
    }
  }


  void bumperCallback(const vacuum_cleaner_pkg::msg::Bumper::SharedPtr msg)
  {
    act_val_bumper = msg -> collision_detected;
    RCLCPP_INFO(this->get_logger(), "act_val_bumper: '%s'", act_val_bumper ? "true" : "false");
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    act_val_yaw = yaw;
  }

  void publish_cmd_vel_function(const float x_val, const float y_val, const float z_val)
  {
      auto twist_msg = geometry_msgs::msg::Twist();

      twist_msg.linear.x = x_val;
      twist_msg.linear.y = y_val;
      twist_msg.angular.z = z_val;
      cmd_vel_publisher_->publish(twist_msg);
      //RCLCPP_INFO(this->get_logger(), "Cmd vel published");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<vacuum_cleaner_pkg::msg::Bumper>::SharedPtr bumper_subscription_;
  rclcpp::TimerBase::SharedPtr timer_main;

  float angle_goal_val;
  float act_val_yaw;
  bool act_val_bumper;
  bool collison_handling;
  const float K_ANGULAR = 1.5;



private:

  float get_random_value_from_range(const float min_value, const float max_value) const
  {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<float> dist(min_value, max_value);
    return dist(mt);
    return 1.0;
  }

};


// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<TurtleBotNode>());
//   rclcpp::shutdown();
//   return 0;
// }