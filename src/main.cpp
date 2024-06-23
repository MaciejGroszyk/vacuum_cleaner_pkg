
#include "rclcpp/rclcpp.hpp"

#include "covering_algorithms/CoveringAlgorithmHandler.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string mode_in = argv[1];
    std::cout << "Mode: " << mode_in << std::endl;

    CoveringAlgorithmHandler cah;

    if      (mode_in == "random")   cah.startRandomWalk();
    else if (mode_in == "spiral")   cah.startSpiralWalk();
    else if (mode_in == "snaking")  cah.startSnakingWalk();

    rclcpp::shutdown();
    return 0;
}