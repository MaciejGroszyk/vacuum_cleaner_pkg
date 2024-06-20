
#include "CoveringAlgorithm.h"

using std::placeholders::_1;

CoveringAlgorithm::CoveringAlgorithm()
{
    // bumper_sim_node = std::make_shared<BumperSensor>();
    rc_node         = std::make_shared<RobotControler>();
    oh_node         = std::make_shared<OdomHandler>();

    rclcpp::executors::MultiThreadedExecutor executor;

    // executor.add_node(bumper_sim_node);
    executor.add_node(rc_node);
    executor.add_node(oh_node);

    executor.spin();
}


// private:
    // virtual void collision_handling() = 0;

    // void rotate_to_angle_(const float angle_goal);

    // void move_forward_(const float x, const float y);

    // float get_random_value_from_range(const float min_value, const float max_value) const
    // {
    //     std::random_device rd;
    //     std::mt19937 mt(rd());
    //     std::uniform_real_distribution<float> dist(min_value, max_value);
    //     return dist(mt);
    // }


    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    // rclcpp::TimerBase::SharedPtr timer_;


};

