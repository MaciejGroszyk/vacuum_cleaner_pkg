#ifndef SPIRALWALK_H
#define SPIRALWALK_H

#include "CoveringAlgorithm.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"


class SpiralWalk : public CoveringAlgorithm
{

public:
    typedef CoveringAlgorithm CoveringAlgorithm;
    std::shared_ptr<LaserHandler> laser_handler_node = std::make_shared<LaserHandler>();
    std::shared_ptr<RobotController> robot_controller_node = std::make_shared<RobotController>();
    std::shared_ptr<OdomHandler> odom_handler_node = std::make_shared<OdomHandler>();

    void algorithm();

    SpiralWalk();
    ~SpiralWalk();
};

#endif