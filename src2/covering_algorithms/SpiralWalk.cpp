#include "SpiralWalk.h"


SpiralWalk::SpiralWalk() : CoveringAlgorithm("spiral_walk")
{
    RCLCPP_INFO(this->get_logger(), "init random walk");
    // this -> timer_algorithm_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&SpiralWalk::algorithm, this));
}

SpiralWalk::~SpiralWalk()
{
    
}

void SpiralWalk::algorithm()
{

}