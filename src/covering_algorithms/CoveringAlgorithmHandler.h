#ifndef COVERINGALGORITHMHANDLER_H
#define COVERINGALGORITHMHANDLER_H

#include "rclcpp/rclcpp.hpp"
#include "RandomWalk.h"
#include "SnakingWalk.h"
#include "SpiralWalk.h"

class CoveringAlgorithmHandler
{
private:
    void start_(std::shared_ptr<CoveringAlgorithm> ca_node);
public:
    CoveringAlgorithmHandler();
    ~CoveringAlgorithmHandler();

    rclcpp::executors::MultiThreadedExecutor executor;

    void startRandomWalk();
    void startSpiralWalk();
    void startSnakingWalk();

};




#endif