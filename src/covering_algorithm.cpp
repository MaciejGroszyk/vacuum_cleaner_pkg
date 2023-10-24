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

using std::placeholders::_1;

class CoveringAlgorithm
{
    CoveringAlgorithm()
    {
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::ServicesQoS());

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 30, std::bind(&TurtleBotNode::odomCallback, this, _1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TurtleBotNode::publish_cmd_vel_function1, this));

    angle_goal_val =  get_random_value_from_range(-3.14, 3.14);
    //RCLCPP_INFO(this->get_logger(), "Publishing random float: '%f'", angle_goal_val);
    }

public:
    virtual void start() = 0;
    virtual void stop() = 0;

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
        //RCLCPP_INFO(this->get_logger(), "Publishing yaw: '%f'", yaw);

        yaw_val = yaw;
        
    }

    void publish_cmd_vel_function1()
    {
        const float K_ANGULAR = 2;
        const float z_vel = (angle_goal_val - yaw_val) * K_ANGULAR;
        publish_cmd_vel_function(0.0, 0.0, z_vel);
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



private:
    virtual void collision_handling() = 0;

    void rotate_to_angle_(const float angle_goal);

    void move_forward_(const float x, const float y);

    float get_random_value_from_range(const float min_value, const float max_value) const
    {
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_real_distribution<float> dist(min_value, max_value);
        return dist(mt);
    }


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;


    float angle_goal_val;
    float yaw_val;

    rclcpp::TimerBase::SharedPtr timer_;


};