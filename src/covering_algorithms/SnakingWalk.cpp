#include "SnakingWalk.h"

SnakingWalk::SnakingWalk() : CoveringAlgorithm("snaking_walk")
{
    RCLCPP_INFO(this->get_logger(), "init snaking walk");
    this->K_ANGULAR = 0.5;

    this ->robot_controller_node -> THETA_VEL_ = 0.25;
}

SnakingWalk::~SnakingWalk()
{

}

void SnakingWalk::algorithm()
{
    if      (state_ == SnakingWalkState::COLLISION_HANDLER) collisionHandler_();
    else if (state_ == SnakingWalkState::MOVE_FORWARD) moveForward_();
    else if (state_ == SnakingWalkState::MOVE_DISTANCE) moveDistance_();
    else if (state_ == SnakingWalkState::ROTATE_LEFT_RIGHT_PREP) rotateLeftRightPrep_();
    else if (state_ == SnakingWalkState::ROTATE_LEFT_RIGHT) rotateLeftRight_();
    else if (state_ == SnakingWalkState::GENERATE_NEW_ANGLE) generateNewAngle_();
    else if (state_ == SnakingWalkState::ROTATE_NEW_ANGLE) rotateNewAngle_();
}


void SnakingWalk::collisionHandler_()
{

}
void SnakingWalk::moveForward_()
{
    turn_num_ = 0;
    this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::FORWARD);

    if (this -> laser_handler_node -> isCollisionFront())
    {
        state_ = SnakingWalkState::ROTATE_LEFT_RIGHT_PREP;
        RCLCPP_INFO(this->get_logger(), "rotate left right prep");
    }
}
void SnakingWalk::moveDistance_()
{
    const bool distance_achieved =  this -> moveDistance(pose_x_, pose_y_, MOVE_DISTANCE_VAL_);

    if (this -> laser_handler_node -> isCollisionThinFront()) 
    {
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::STOP);
        state_ = SnakingWalkState::GENERATE_NEW_ANGLE;
        RCLCPP_INFO(this->get_logger(), "generate new angle");
    }
    else if (distance_achieved)
    {
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::STOP);
        state_ = SnakingWalkState::ROTATE_LEFT_RIGHT_PREP;
        RCLCPP_INFO(this->get_logger(), "rotate left right prep");
    }
}

void SnakingWalk::rotateLeftRightPrep_()
{

    angle_goal_ = this-> odom_handler_node -> act_val_yaw + (sign_() * 1.57);

    if (angle_goal_ > 3.14)
        angle_goal_ = -3.14 + (angle_goal_ - 3.14);
    else if (angle_goal_ < -3.14)
        angle_goal_ = 3.14 + (angle_goal_ - (-3.14));
    state_ = SnakingWalkState::ROTATE_LEFT_RIGHT;
    RCLCPP_INFO(this->get_logger(), "rotate left right");

    if (right_left_)
    {
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::RIGHT);
    }
    else
    {
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::LEFT);
    }
}

void SnakingWalk::rotateLeftRight_()
{
    // const bool angle_goal_achieved_ = this-> rotateToAngle(angle_goal_);
    bool angle_goal_achieved_;
    if (abs(this-> odom_handler_node -> act_val_yaw - angle_goal_) < 0.05)
        angle_goal_achieved_ = true;
    else
        angle_goal_achieved_ = false;

    if (angle_goal_achieved_)
    {
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::STOP);
        ++turn_num_;

        if (turn_num_ < 2)
        {
            pose_x_ = odom_handler_node -> act_val_x;
            pose_y_ = odom_handler_node -> act_val_y;
            state_ = SnakingWalkState::MOVE_DISTANCE;
            RCLCPP_INFO(this->get_logger(), "move distance");
        }
        else
        {
            state_ = SnakingWalkState::MOVE_FORWARD;
            RCLCPP_INFO(this->get_logger(), "move foward");
            right_left_ = not right_left_;
            turn_num_ = 0;
        }
            
    }
    // else if(angle_goal_achieved_ && this -> laser_handler_node -> isCollisionFront())
    // {
    //     state_ = SnakingWalkState::GENERATE_NEW_ANGLE;
    // }
}
void SnakingWalk::generateNewAngle_()
{
    turn_num_ = 0;
    this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::STOP);

    angle_goal_ = this -> generateNewAngleGoal();

    state_ = SnakingWalkState::ROTATE_NEW_ANGLE;
    RCLCPP_INFO(this->get_logger(), "rotateNewAngle_");
}

void SnakingWalk::rotateNewAngle_()
{
    const bool angle_goal_achieved_ = this-> rotateToAngle(angle_goal_);
    if (angle_goal_achieved_ && not this -> laser_handler_node -> isCollisionFront())
    {
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::STOP);
        state_ = SnakingWalkState::MOVE_FORWARD;
        RCLCPP_INFO(this->get_logger(), "move foward");
    }
    else if(angle_goal_achieved_ && this -> laser_handler_node -> isCollisionFront())
    {
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::STOP);
        state_ = SnakingWalkState::GENERATE_NEW_ANGLE;
        RCLCPP_INFO(this->get_logger(), "generate new angle");
    }
}

int SnakingWalk::sign_() const
{
    return right_left_ ? -1 : 1; 
}