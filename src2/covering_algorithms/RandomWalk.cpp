#include "RandomWalk.h"


RandomWalk::RandomWalk() : CoveringAlgorithm("random_walk")
{
    RCLCPP_INFO(this->get_logger(), "init random walk");
    // this->timer_algorithm_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&RandomWalk::algorithm, this));
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
    // RCLCPP_INFO(this->get_logger(), "random walk");
    if (this -> lh_node -> isCollisionFront() && (not collison_handling_))
    {
        // RCLCPP_INFO(this->get_logger(), "new angle");
        angle_goal_ = this -> generateNewAngleGoal();
        this -> rc_node -> move(this -> rc_node -> MoveCommands::STOP);
        collison_handling_ = true;
    }
    else if (collison_handling_)
    {
        // RCLCPP_INFO(this->get_logger(), "Collision handling");
        collison_handling_ = not (this-> rotateToAngle(angle_goal_));
    }
    else if ((not (this -> lh_node -> isCollisionFront())) && (not collison_handling_))
    {
        // RCLCPP_INFO(this->get_logger(), "Publishing cmd vel");
        this -> rc_node -> move(this -> rc_node -> MoveCommands::FORWARD);
    }     
}