#include "LaserHandler.h"

LaserHandler::LaserHandler()
    : Node("bumber_sensor_sim")
    {
    scanner_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 30, std::bind(&LaserHandler::laser_callback, this, _1));

    bumper_publisher_ = this->create_publisher<vacuum_cleaner_pkg::msg::Bumper>("/bumper_sensor", 10);

    }

void LaserHandler::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_in)
{
    // RCLCPP_INFO(this->get_logger(), "laser_handler_node");
    const float min_front_laser_range = get_min_front_laser_range(msg_in);
    const float min_left_laser_range = get_min_left_laser_range(msg_in);
    const float min_right_laser_range = get_min_right_laser_range(msg_in);
    auto message = vacuum_cleaner_pkg::msg::Bumper();

    front_detected_ = isCollisionFront(min_front_laser_range);
    left_detected_  = isCollision(min_left_laser_range);
    right_detected_ = isCollision(min_right_laser_range);
    message.front_detected = front_detected_;
    message.left_detected  = left_detected_;
    message.right_detected = right_detected_;
    bumper_publisher_ -> publish(message);
}

bool LaserHandler::isCollisionFront(const float &bumper_val) const
{
    return COLLISION_DISTANCE_FRONT > bumper_val ? true : false;
}
bool  LaserHandler::isCollisionFront() const
{
    return front_detected_;
}

bool  LaserHandler::isCollisionLeft() const
{
    return left_detected_;
}

bool  LaserHandler::isCollisionRight() const
{
    return right_detected_;
}

bool LaserHandler::isCollision(const float &bumper_val) const
{
    return COLLISION_DISTANCE > bumper_val ? true : false;
}

bool  LaserHandler::isCollision() const
{
    return front_detected_ || left_detected_ || right_detected_;
}

float LaserHandler::get_min_front_laser_range(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const 
{
    const std::vector<float> v = msg_in -> ranges;
    const auto RANGES_SIZE = v.end() - v.begin();
    
    const float min_left_front = *std::min_element(v.begin(), v.begin() + RANGES_SIZE/SCALE);
    const float min_right_front = *std::min_element(v.end() - RANGES_SIZE/SCALE, v.end()-1);

    return (min_left_front < min_right_front ? min_left_front : min_right_front);
}

float LaserHandler::get_min_left_laser_range(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const 
{
    const std::vector<float> v = msg_in -> ranges;
    const auto RANGES_SIZE = v.end() - v.begin();

    return *std::min_element(v.begin() + RANGES_SIZE/4 - RANGES_SIZE/SCALE, v.begin() + RANGES_SIZE/4 + RANGES_SIZE/SCALE);
}

float LaserHandler::get_min_right_laser_range(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const 
{
    const std::vector<float> v = msg_in -> ranges;
    const auto RANGES_SIZE = v.end() - v.begin();

    return *std::min_element(v.end() - RANGES_SIZE/4 - RANGES_SIZE/SCALE, v.end() - RANGES_SIZE/4 + RANGES_SIZE/SCALE);
}

