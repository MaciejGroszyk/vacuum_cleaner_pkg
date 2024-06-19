
#include "rclcpp/rclcpp.hpp"
#include "CoveringAlgorithm.h"
#include "snaking.cpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    Snaking snak_alg;


    rclcpp::shutdown();
    return 0;
}