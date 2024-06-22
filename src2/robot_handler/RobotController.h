#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class RobotController : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

public:
    RobotController();
    // RobotController(const std::string & namespace_ = "");

    ~RobotController();

    enum MoveCommands
    {
        STOP,
        LEFT,
        RIGHT,
        FORWARD
    };
    void move(const MoveCommands&);
    void move(const float x_val, const float y_val, const float z_val);
    void publish_cmd_vel_function(const float, const float, const float);
};

#endif