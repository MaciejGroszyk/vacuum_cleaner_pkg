#ifndef COVERINGALGORITHM_H
#define COVERINGALGORITHM_H

#include <random>
#include "../robot_handler/RobotController.h"
#include "../robot_handler/OdomHandler.h"
#include "../robot_handler/LaserHandler.h"
// #include "../src/bumper_sim.cpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

class CoveringAlgorithm : public rclcpp::Node
{
private:
    const float K_ANGULAR = 1.5;
    float get_random_value_from_range(const float min_value, const float max_value) const;

    rclcpp::TimerBase::SharedPtr timer_algorithm_;

public:
    CoveringAlgorithm(const std::string &node_name_, const std::string &namespace_ = "");
    ~CoveringAlgorithm();

    std::shared_ptr<LaserHandler> lh_node = std::make_shared<LaserHandler>();
    std::shared_ptr<RobotController> rc_node = std::make_shared<RobotController>();
    std::shared_ptr<OdomHandler> oh_node = std::make_shared<OdomHandler>();
    
    float generate_new_angle_goal() const;
    bool rotate_to_angle(const float angle_goal);


    virtual void algorithm() = 0;
};


#endif