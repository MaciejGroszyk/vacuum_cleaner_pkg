
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
    float z_vel = std::min((angle_goal - odom_handler_node -> act_val_yaw) * K_ANGULAR, 0.25);
    if (abs(z_vel) < 0.05)
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

bool CoveringAlgorithm::moveDistance(const float x_pose, const float y_pose, const float distance)
{
    const float x = odom_handler_node -> act_val_x;
    const float y = odom_handler_node -> act_val_y;
    const float distance_done = calcDistance_(x_pose, y_pose, x, y);
    if (distance_done > distance - 0.1)
    {
        robot_controller_node -> move(robot_controller_node -> MoveCommands::STOP);
        return true;
    }
    else
    {
        const float X_VEL = std::min(robot_controller_node ->getXVelocity(), (abs(distance_done - distance))*K_LINEAR);
        robot_controller_node -> move(X_VEL, 0.0, 0.0);
        return false;
    }
}


float CoveringAlgorithm::calcDistance_(const float x_prev, const float y_prev, const float x, const float y) const
{
    return sqrt(pow(x - x_prev, 2) + pow(y -y_prev, 2));
}

float CoveringAlgorithm::getRandomValueFromRange_(const float min_value, const float max_value) const
{
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<float> dist(min_value, max_value);
    return dist(mt);
}

float CoveringAlgorithm::generateNewAngleGoal() const
{
    return getRandomValueFromRange_(-3.14, 3.14);
}


