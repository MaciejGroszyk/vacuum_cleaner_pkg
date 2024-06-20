#ifndef COVERINGALGORITHMHANDLER_H
#define COVERINGALGORITHMHANDLER_H

#include "rclcpp/rclcpp.hpp"
#include "RandomWalk.h"

class CoveringAlgorithmHandler
{
private:
    /* data */
public:
    CoveringAlgorithmHandler(/* args */);
    ~CoveringAlgorithmHandler();
    rclcpp::executors::MultiThreadedExecutor executor;
    void startRandomWalk();

};




#endif