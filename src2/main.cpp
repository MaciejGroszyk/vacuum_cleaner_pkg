
#include "rclcpp/rclcpp.hpp"

#include "covering_algorithms/CoveringAlgorithm.h"
#include "covering_algorithms/snaking.cpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    Snaking snak_alg;


    rclcpp::shutdown();
    return 0;
}