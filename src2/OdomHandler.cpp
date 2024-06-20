#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

OdomHandler::OdomHandler(): Node("odometry_handler")
{
odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 20, std::bind(&TurtleBotControl::odomCallback, this, _1));
}

void OdomHandler::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

