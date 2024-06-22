#ifndef LASERHANDLER_H
#define LASERHANDLER_H


#include "rclcpp/rclcpp.hpp"
#include "vacuum_cleaner_pkg/msg/bumper.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rcutils/logging_macros.h"
using std::placeholders::_1;

class LaserHandler : public rclcpp::Node
{

private:
    const uint SCALE_ = 12;
    const float COLLISION_DISTANCE_FRONT_ = 0.6;
    const float COLLISION_DISTANCE_ = 0.3;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanner_subscription_;
    rclcpp::Publisher<vacuum_cleaner_pkg::msg::Bumper>::SharedPtr bumper_publisher_;

    bool front_detected_;
    bool left_detected_;
    bool right_detected_;

public:
    LaserHandler();
    bool isCollisionFront() const;
    bool isCollisionLeft() const;
    bool isCollisionRight() const;
    bool isCollision() const;

private:
    void laserCallback_(const sensor_msgs::msg::LaserScan::SharedPtr msg_in);


    bool isCollisionFront(const float &bumper_val) const;
    bool isCollision(const float &bumper_val) const;

    float getMinFrontLaserRange_(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const;
    float getMinLeftLaserRange_(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const;
    float getMinRightLaserRange_(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const;   
};

#endif