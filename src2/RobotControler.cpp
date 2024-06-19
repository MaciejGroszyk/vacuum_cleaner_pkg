
#include "RobotControler.h"

using std::placeholders::_1;

RobotControler::RobotControler()
    : Node("robot_controler")
{
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::ServicesQoS());
}

RobotControler::~RobotControler()
{
    
}

void RobotControler::move(const RobotControler::MoveCommands & move_command)
{
    switch(move_command)
    {
        case STOP       : publish_cmd_vel_function(0.0, 0.0, 0.0); break;
        case LEFT       : publish_cmd_vel_function(0.0, 0.0, 0.1); break;
        case RIGHT      : publish_cmd_vel_function(0.0, 0.0,-0.1); break;
        case FORWARD    : publish_cmd_vel_function(0.5, 0.0, 0.0); break;
    }
}

void RobotControler::publish_cmd_vel_function(const float x_val, const float y_val, const float z_val)
{
    auto twist_msg = geometry_msgs::msg::Twist();

    twist_msg.linear.x = x_val;
    twist_msg.linear.y = y_val;
    twist_msg.angular.z = z_val;
    cmd_vel_publisher_->publish(twist_msg);
    //RCLCPP_INFO(this->get_logger(), "Cmd vel published");
}

// class RobotControler : public rclcpp::Node
// {

// private:
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

//     void publish_cmd_vel_function(const float x_val, const float y_val, const float z_val)
//     {
//         auto twist_msg = geometry_msgs::msg::Twist();

//         twist_msg.linear.x = x_val;
//         twist_msg.linear.y = y_val;
//         twist_msg.angular.z = z_val;
//         cmd_vel_publisher_->publish(twist_msg);
//         //RCLCPP_INFO(this->get_logger(), "Cmd vel published");
//     }

// public:
//     RobotControler(): Node("robot_controler")
//     {
//         cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::ServicesQoS());
//     }

//     void move(const MoveCommands & move_command)
//     {
//         switch(move_command)
//         {
//             case STOP       : publish_cmd_vel_function(0.0, 0.0, 0.0); break;
//             case LEFT       : publish_cmd_vel_function(0.0, 0.0, 0.1); break;
//             case RIGHT      : publish_cmd_vel_function(0.0, 0.0,-0.1); break;
//             case FORWARD    : publish_cmd_vel_function(0.5, 0.0, 0.0); break;
//         }
//     }
// };
