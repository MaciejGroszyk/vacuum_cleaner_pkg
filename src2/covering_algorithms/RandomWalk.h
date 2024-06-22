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

public:
    RandomWalk();
    ~RandomWalk();
    void start();
    void stop();
};

#endif