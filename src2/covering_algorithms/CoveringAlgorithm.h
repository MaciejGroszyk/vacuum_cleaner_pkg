#ifndef COVERINGALGORITHM_H
#define COVERINGALGORITHM_H

#include <random>
#include <cmath> 

#include "../robot_handler/RobotController.h"
#include "../robot_handler/OdomHandler.h"
#include "../robot_handler/LaserHandler.h"
// #include "../src/bumper_sim.cpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

class CoveringAlgorithm : public rclcpp::Node
{
private:
    float getRandomValueFromRange_(const float min_value, const float max_value) const;
    float calcDistance_(const float x_prev, const float y_prev, const float x, const float y) const;

    rclcpp::TimerBase::SharedPtr timer_algorithm_;

public:
    float K_ANGULAR = 1.5;
    float K_LINEAR = 1.5;
    CoveringAlgorithm(const std::string &node_name_, const std::string &namespace_ = "");
    ~CoveringAlgorithm();

    std::shared_ptr<LaserHandler> laser_handler_node = std::make_shared<LaserHandler>();
    std::shared_ptr<RobotController> robot_controller_node = std::make_shared<RobotController>();
    std::shared_ptr<OdomHandler> odom_handler_node = std::make_shared<OdomHandler>();
    
    float generateNewAngleGoal() const;
    bool rotateToAngle(const float angle_goal);
    bool moveDistance(const float x_pose, const float y_pose, const float distance);


    virtual void algorithm() = 0;
};


#endif