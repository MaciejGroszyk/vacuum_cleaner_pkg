#include "RandomWalk.h"


RandomWalk::RandomWalk() : CoveringAlgorithm(), Node("random_walk")
{
    RCLCPP_INFO(this->get_logger(), "init random walk");
    timer_main_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&RandomWalk::algorithm, this));
}

RandomWalk::~RandomWalk()
{

}

void RandomWalk::start()
{
    RCLCPP_INFO(this->get_logger(), "start");
}

void RandomWalk::algorithm()
{
    RCLCPP_INFO(this->get_logger(), "random walk");
    if (this -> lh_node -> isCollisionFront() && (not collison_handling_))
    {
        RCLCPP_INFO(this->get_logger(), "new angle");
        angle_goal_ = this -> generate_new_angle_goal();
        collison_handling_ = true;
    }
    else if (collison_handling_)
    {
        RCLCPP_INFO(this->get_logger(), "Collision handling");
        collison_handling_ = not (rotate_to_angle(angle_goal_));
    }
    else if ((not (this -> lh_node -> isCollisionFront())) && (not collison_handling_))
    {
        RCLCPP_INFO(this->get_logger(), "Publishing cmd vel");
        this -> rc_node -> move(this -> rc_node -> MoveCommands::FORWARD);
    }     
}

bool RandomWalk::rotate_to_angle(const float angle_goal)
{
    float z_vel = (angle_goal - oh_node -> act_val_yaw) * K_ANGULAR;
    if (abs(z_vel) < 0.3)
    {
        rc_node -> move(rc_node -> MoveCommands::STOP);
        return true;
    }
    else
    {
        rc_node -> publish_cmd_vel_function(0.0, 0.0, z_vel);
        return false;
    }
}