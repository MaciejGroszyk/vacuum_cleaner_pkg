#include "SpiralWalk.h"


SpiralWalk::SpiralWalk() : CoveringAlgorithm("spiral_walk")
{
    RCLCPP_INFO(this->get_logger(), "init spiral walk");
}

SpiralWalk::~SpiralWalk()
{
    
}

void SpiralWalk::algorithm()
{
    if ( (this-> laser_handler_node-> isCollisionFront()) && (not collision_handling_))
    {
        angle_goal_ = this -> generateNewAngleGoal();
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::STOP);
        collision_handling_ = true;
    }
    else if (collision_handling_)
    {
        spiral_num_ = 0;
        collision_handling_ = not (this-> rotateToAngle(angle_goal_));
        move_from_wall_handling_ = true;
        move_from_wall_time_ = this->get_clock()->now().seconds();
    }
    else if (move_from_wall_handling_)
    {
        this -> robot_controller_node -> move(this -> robot_controller_node -> MoveCommands::FORWARD);
        const double time_now =  this->get_clock()->now().seconds();

        const float VEL_X = this -> robot_controller_node -> getXVelocity();
        if (VEL_X * (time_now -move_from_wall_time_) > MOVE_FROM_WALL_DISTANCE_)
            move_from_wall_handling_ = false;
    }
    else if ((not this-> laser_handler_node-> isCollisionFront()) && (not collision_handling_))
    {
        const int angle_num = getActualQuarterNum_();
        quarters_visited_[angle_num] = true;

        if (isAllQuartersVisited_())
        {
            spiral_num_ += 1.0;
            quarters_visited_[0] = false;
            quarters_visited_[1] = false;
            quarters_visited_[2] = false;
            quarters_visited_[3] = false;
        }

        const float z_vel = START_YAW_VEL_ - DECREMENT_YAW_VEL_BY_ * spiral_num_;
        this -> robot_controller_node -> move(0.35, 0.0, z_vel);
    }   
}

bool SpiralWalk::isAllQuartersVisited_() const
{
    return std::all_of(quarters_visited_.begin(), quarters_visited_.end(), [](bool v) { return v; });
}

int SpiralWalk::getActualQuarterNum_() const
{
    //1 | 0
    //-----
    //2 | 3
    const int q_num = int(std::ceil( this->odom_handler_node -> act_val_yaw/ float(M_PI/2) )) + 1;
    return abs(q_num) > 3 ? 4 : q_num;
}
