#include "RandomWalk.h"


RandomWalk::RandomWalk() : CoveringAlgorithm("random_walk")
{
    RCLCPP_INFO(this->get_logger(), "init random walk");
    // this->timer_algorithm_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&RandomWalk::algorithm, this));
}

RandomWalk::~RandomWalk()
{

}

void RandomWalk::algorithm()
{
    // RCLCPP_INFO(this->get_logger(), "random walk");
    if (this -> laser_handler_node -> isCollisionFront() && (not collison_handling_))
    {
        // RCLCPP_INFO(this->get_logger(), "new angle");
        angle_goal_ = this -> generateNewAngleGoal();
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::STOP);
        collison_handling_ = true;
    }
    else if (collison_handling_)
    {
        // RCLCPP_INFO(this->get_logger(), "Collision handling");
        collison_handling_ = not (this-> rotateToAngle(angle_goal_));
    }
    else if ((not (this -> laser_handler_node -> isCollisionFront())) && (not collison_handling_))
    {
        // RCLCPP_INFO(this->get_logger(), "Publishing cmd vel");
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::FORWARD);
    }     
}
