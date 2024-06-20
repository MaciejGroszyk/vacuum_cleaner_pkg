#ifndef ODOMHANDLER_H
#define ODOMHANDLER_H

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

class OdomHandler : public rclcpp::Node
{

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double act_val_yaw;

public:
    OdomHandler();

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};


#endif