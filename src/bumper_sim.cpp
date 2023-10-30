#include <memory>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "vacuum_cleaner_pkg/msg/bumper.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

class BumperSensor : public rclcpp::Node
{
  //// bumper sensor simulator acording to data from front laser scaner data
  public:
    BumperSensor()
    : Node("bumber_sensor_sim")
    {
      scaner_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&BumperSensor::topic_callback, this, _1));

      bumper_publisher_ = this->create_publisher<vacuum_cleaner_pkg::msg::Bumper>("/bumper_sensor", 10);

    }

  private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_in)
    {
      const float min_front_laser_range = get_min_front_laser_range(msg_in);

      if (min_front_laser_range < COLLISION_DISTANCE)
      {
        auto message = vacuum_cleaner_pkg::msg::Bumper();
        message.collision_detected = true;
        bumper_publisher_ -> publish(message);
        RCLCPP_INFO(this->get_logger(), "Bumper sensor active. Collision detected on range:'%f'", min_front_laser_range);
      }

    }

    float get_min_front_laser_range(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const 
    {
        const std::vector<float> v = msg_in -> ranges;
        const auto RANGES_SIZE = v.end() - v.begin();
        const uint SCALE = 12;

        const float min_left_front = *std::min_element(v.begin(), v.begin() + RANGES_SIZE/SCALE);
        const float min_right_front = *std::min_element(v.end() - RANGES_SIZE/SCALE, v.end()-1);

        return (min_left_front < min_right_front ? min_left_front : min_right_front);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scaner_subscription_;
    rclcpp::Publisher<vacuum_cleaner_pkg::msg::Bumper>::SharedPtr bumper_publisher_;

    const float COLLISION_DISTANCE = 0.35;
};

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<BumperSensor>());
//   rclcpp::shutdown();
//   return 0;
// }