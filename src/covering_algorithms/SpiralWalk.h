#ifndef SPIRALWALK_H
#define SPIRALWALK_H

#include "CoveringAlgorithm.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"


class SpiralWalk : public CoveringAlgorithm
{
private:
    // spiral_move_config
    const float DECREMENT_YAW_VEL_BY_ = 0.2;
    const float START_YAW_VEL_ = 1.2;
    const float MOVE_FROM_WALL_DISTANCE_ = 2;

    std::vector<bool> quarters_visited_ = std::vector<bool>(4); 
    float spiral_num_ = 0.0;
    float angle_goal_;
    double move_from_wall_time_;

    bool front_detected_;
    bool right_detected_;
    bool left_detected_;
    bool collision_handling_;
    bool move_from_wall_handling_;

    bool isAllQuartersVisited_() const;
    int getActualQuarterNum_() const;

public:
    void algorithm();

    SpiralWalk();
    ~SpiralWalk();
};

#endif