
#include "CoveringAlgorithm.h"

using std::placeholders::_1;


CoveringAlgorithm::CoveringAlgorithm(const std::string &node_name_, const std::string &namespace_) 
    : Node(node_name_, namespace_)
{
    // RCLCPP_INFO(this->get_logger(), "init cov alg");
    // lh_node = std::make_shared<LaserHandler>();
    // rc_node = std::make_shared<RobotController>();
    // oh_node = std::make_shared<OdomHandler>();

    // rclcpp::executors::MultiThreadedExecutor executor;
    this->timer_algorithm_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&CoveringAlgorithm::algorithm, this));
}

CoveringAlgorithm::~CoveringAlgorithm()
{

}

bool CoveringAlgorithm::rotate_to_angle(const float angle_goal)
{
    float z_vel = (angle_goal - oh_node -> act_val_yaw) * K_ANGULAR;
    if (abs(z_vel) < 0.3)
    {
        rc_node -> move(rc_node -> MoveCommands::STOP);
        return true;
    }
    else
    {
        rc_node -> publish_cmd_vel_function(0.0, 0.0, z_vel);
        return false;
    }
}

float CoveringAlgorithm::get_random_value_from_range(const float min_value, const float max_value) const
{
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<float> dist(min_value, max_value);
    return dist(mt);
}

float CoveringAlgorithm::generate_new_angle_goal() const
{
    return get_random_value_from_range(-3.14, 3.14);
}


