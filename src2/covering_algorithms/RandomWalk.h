#ifndef RANDOMWALK_H
#define RANDOMWALK_H


#include "CoveringAlgorithm.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

class RandomWalk : public CoveringAlgorithm
{
private:
    bool collison_handling_;
    float angle_goal_;

    void algorithm();
    // rclcpp::TimerBase::SharedPtr timer_main_;
    // bool rotate_to_angle(const float angle_goal);
public:
    // std::shared_ptr<LaserHandler>   lh_node = std::make_shared<LaserHandler>();
    // std::shared_ptr<RobotController> rc_node = std::make_shared<RobotController>();
    // std::shared_ptr<OdomHandler>    oh_node = std::make_shared<OdomHandler>();

    RandomWalk();
    ~RandomWalk();
    void start();
    void stop();
};


#endif