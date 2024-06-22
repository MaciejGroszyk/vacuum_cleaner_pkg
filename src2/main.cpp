
#include "rclcpp/rclcpp.hpp"

#include "covering_algorithms/CoveringAlgorithmHandler.h"
#include "covering_algorithms/CoveringAlgorithm.h"
#include "covering_algorithms/snaking.cpp"
#include "covering_algorithms/RandomWalk.h"
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string mode_in = argv[1];
    std::cout << "Mode: " << mode_in << std::endl;

    CoveringAlgorithmHandler cah;

    if (mode_in == "random") cah.startRandomWalk();
    else if (mode_in == "spiral") cah.startSpiralWalk();

    rclcpp::shutdown();
    return 0;
}