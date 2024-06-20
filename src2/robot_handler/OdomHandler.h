#ifndef ODOMHANDLER_H
#define ODOMHANDLER_H

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

class OdomHandler : public rclcpp::Node
{

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    

public:
    OdomHandler();
    double act_val_yaw;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};


#endif