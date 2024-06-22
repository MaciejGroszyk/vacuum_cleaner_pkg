#ifndef ODOMHANDLER_H
#define ODOMHANDLER_H

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "rcutils/logging_macros.h"
class OdomHandler : public rclcpp::Node
{

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odomCallback_(const nav_msgs::msg::Odometry::SharedPtr msg);

public:
    OdomHandler();
    double act_val_yaw;
};


#endif