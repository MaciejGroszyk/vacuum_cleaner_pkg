#ifndef COVERINGALGORITHM_H
#define COVERINGALGORITHM_H

#include "RobotControler.h"
// #include "../src/bumper_sim.cpp"
#include "rclcpp/rclcpp.hpp"


class CoveringAlgorithm
{

private:
    // std::shared_ptr<BumperSensor> bumper_sim_node;
    std::shared_ptr<RobotControler> rc_node;
};

#endif