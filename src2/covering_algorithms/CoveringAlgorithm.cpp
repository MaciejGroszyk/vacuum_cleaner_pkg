
#include "CoveringAlgorithm.h"

using std::placeholders::_1;


CoveringAlgorithm::CoveringAlgorithm(const std::string &node_name_, const std::string &namespace_) 
    : Node(node_name_, namespace_)
{
    this->timer_algorithm_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&CoveringAlgorithm::algorithm, this));
}

CoveringAlgorithm::~CoveringAlgorithm()
{

}

bool CoveringAlgorithm::rotateToAngle(const float angle_goal)
{
    float z_vel = (angle_goal - odom_handler_node -> act_val_yaw) * K_ANGULAR;
    if (abs(z_vel) < 0.3)
    {
        robot_controller_node -> move(robot_controller_node -> MoveCommands::STOP);
        return true;
    }
    else
    {
        robot_controller_node -> move(0.0, 0.0, z_vel);
        return false;
    }
}

float CoveringAlgorithm::getRandomValueFromRange(const float min_value, const float max_value) const
{
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<float> dist(min_value, max_value);
    return dist(mt);
}

float CoveringAlgorithm::generateNewAngleGoal() const
{
    return getRandomValueFromRange(-3.14, 3.14);
}


