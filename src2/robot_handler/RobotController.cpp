
#include "RobotController.h"

using std::placeholders::_1;

RobotController::RobotController()
    : Node("robot_controler")
{
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::ServicesQoS());
}

RobotController::~RobotController()
{
    
}

void RobotController::move(const RobotController::MoveCommands & move_command)
{
    switch(move_command)
    {
        case STOP       : move(0.0, 0.0, 0.0); break;
        case LEFT       : move(0.0, 0.0, THETA_VEL_); break;
        case RIGHT      : move(0.0, 0.0,-THETA_VEL_); break;
        case FORWARD    : move(X_VEL_, 0.0, 0.0); break;
    }
}

void RobotController::move(const float x_val, const float y_val, const float z_val)
{
    auto twist_msg = geometry_msgs::msg::Twist();

    twist_msg.linear.x = x_val;
    twist_msg.linear.y = y_val;
    twist_msg.angular.z = z_val;
    cmd_vel_publisher_->publish(twist_msg);
    //RCLCPP_INFO(this->get_logger(), "Cmd vel published");
}

float RobotController::getXVelocity() const
{
    return X_VEL_;
}
float RobotController::getThetaVelocity() const
{
    return THETA_VEL_;
}