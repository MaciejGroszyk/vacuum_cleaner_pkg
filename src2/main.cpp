
#include "rclcpp/rclcpp.hpp"

#include "covering_algorithms/CoveringAlgorithmHandler.h"
#include "covering_algorithms/CoveringAlgorithm.h"
#include "covering_algorithms/snaking.cpp"
#include "covering_algorithms/RandomWalk.h"
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Snaking snak_alg;
    // auto rw_node = std::make_shared<RandomWalk>();
    // rw_node -> start();  

    // RandomWalk randomWalk;
    CoveringAlgorithmHandler cah;
    cah.startRandomWalk();

    rclcpp::shutdown();
    return 0;
}