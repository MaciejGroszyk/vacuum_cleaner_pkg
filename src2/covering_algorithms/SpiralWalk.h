#ifndef SPIRALWALK_H
#define SPIRALWALK_H

#include "CoveringAlgorithm.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"


class SpiralWalk : public CoveringAlgorithm
{

public:
    void algorithm();

    SpiralWalk();
    ~SpiralWalk();
};

#endif