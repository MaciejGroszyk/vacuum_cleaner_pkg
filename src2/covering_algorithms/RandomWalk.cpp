#include "RandomWalk.h"


RandomWalk::RandomWalk() : CoveringAlgorithm("random_walk")
{
    RCLCPP_INFO(this->get_logger(), "init random walk");
}

RandomWalk::~RandomWalk()
{

}

void RandomWalk::algorithm()
{
    // RCLCPP_INFO(this->get_logger(), "random walk");
    if (this -> laser_handler_node -> isCollisionFront() && (not collision_handling_))
    {
        // RCLCPP_INFO(this->get_logger(), "new angle");
        angle_goal_ = this -> generateNewAngleGoal();
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::STOP);
        collision_handling_ = true;
    }
    else if (collision_handling_)
    {
        // RCLCPP_INFO(this->get_logger(), "Collision handling");
        collision_handling_ = not (this-> rotateToAngle(angle_goal_));
    }
    else if ((not (this -> laser_handler_node -> isCollisionFront())) && (not collision_handling_))
    {
        // RCLCPP_INFO(this->get_logger(), "Publishing cmd vel");
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::FORWARD);
    }     
}
