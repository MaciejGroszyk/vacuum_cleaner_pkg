#ifndef LASERHANDLER_H
#define LASERHANDLER_H


#include "rclcpp/rclcpp.hpp"
#include "vacuum_cleaner_pkg/msg/bumper.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

class LaserHandler : public rclcpp::Node
{

private:
    const uint SCALE = 12;
    const float COLLISION_DISTANCE_FRONT = 0.55;
    const float COLLISION_DISTANCE = 0.2;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scaner_subscription_;
    rclcpp::Publisher<vacuum_cleaner_pkg::msg::Bumper>::SharedPtr bumper_publisher_;

public:
    LaserHandler();

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_in);


    bool isCollisionFront(const float &bumper_val) const;
    bool isCollision(const float &bumper_val) const;

    float get_min_front_laser_range(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const;
    float get_min_left_laser_range(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const;
    float get_min_right_laser_range(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const;   
};

#endif