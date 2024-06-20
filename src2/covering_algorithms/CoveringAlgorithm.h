#ifndef COVERINGALGORITHM_H
#define COVERINGALGORITHM_H

#include <random>
#include "../robot_handler/RobotControler.h"
#include "../robot_handler/OdomHandler.h"
#include "../robot_handler/LaserHandler.h"
// #include "../src/bumper_sim.cpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

class CoveringAlgorithm
{
private:

    float get_random_value_from_range(const float min_value, const float max_value) const;
public:
    const float K_ANGULAR = 1.5;
    CoveringAlgorithm();
    ~CoveringAlgorithm();

    // std::shared_ptr<LaserHandler>   lh_node = std::make_shared<LaserHandler>();
    // std::shared_ptr<RobotControler> rc_node = rc_node = std::make_shared<RobotControler>();
    // std::shared_ptr<OdomHandler>    oh_node = std::make_shared<OdomHandler>();
    

    float generate_new_angle_goal() const;
    // bool rotate_to_angle(const float angle_goal);
};


#endif