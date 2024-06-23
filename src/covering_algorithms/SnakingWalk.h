#ifndef SNAKINGWALK_H
#define SNAKINGWALK_H

#include "CoveringAlgorithm.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

class SnakingWalk : public CoveringAlgorithm
{
private:
    float angle_goal_;
    float pose_x_;
    float pose_y_;
    const float MOVE_DISTANCE_VAL_ = 0.5;

    bool right_left_ = true; //if true turn right, else turn left
    int turn_num_ = 0; // 0, max value 2

    int sign_() const;

    void collisionHandler_();
    void moveForward_();
    void moveDistance_();
    void rotateLeftRightPrep_();
    void rotateLeftRight_();
    void generateNewAngle_();
    void rotateNewAngle_();

    int state_ = SnakingWalkState::MOVE_FORWARD;

public:
    SnakingWalk();
    ~SnakingWalk();


    enum SnakingWalkState {
        COLLISION_HANDLER = 0,
        MOVE_FORWARD = 10,
        MOVE_DISTANCE = 20,
        ROTATE_LEFT_RIGHT_PREP =30,
        ROTATE_LEFT_RIGHT = 40,
        GENERATE_NEW_ANGLE = 50,
        ROTATE_NEW_ANGLE = 60
    };


    void algorithm();
};

#endif