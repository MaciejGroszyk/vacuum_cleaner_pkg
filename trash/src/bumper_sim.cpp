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
  private:
    const uint SCALE = 12;
    const float COLLISION_DISTANCE_FRONT = 0.55; //0.6
    const float COLLISION_DISTANCE = 0.2;

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
      const float min_left_laser_range = get_min_left_laser_range(msg_in);
      const float min_right_laser_range = get_min_right_laser_range(msg_in);
      auto message = vacuum_cleaner_pkg::msg::Bumper();


      message.front_detected = isCollisionFront(min_front_laser_range);
      message.left_detected = isCollision(min_left_laser_range);
      message.right_detected = isCollision(min_right_laser_range);
      // RCLCPP_INFO(this->get_logger(), "Left:'%b'", message.left_detected);
      // RCLCPP_INFO(this->get_logger(), "Right:'%b'", message.right_detected);
      // RCLCPP_INFO(this->get_logger(), "Front:'%b'", message.front_detected);
      bumper_publisher_ -> publish(message);

    }

    bool isCollisionFront(const float &bumper_val) const
    {
      return COLLISION_DISTANCE_FRONT > bumper_val ? true : false;
    }

    bool isCollision(const float &bumper_val) const
    {
      return COLLISION_DISTANCE > bumper_val ? true : false;
    }

    float get_min_front_laser_range(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const 
    {
        const std::vector<float> v = msg_in -> ranges;
        const auto RANGES_SIZE = v.end() - v.begin();
        

        const float min_left_front = *std::min_element(v.begin(), v.begin() + RANGES_SIZE/SCALE);
        const float min_right_front = *std::min_element(v.end() - RANGES_SIZE/SCALE, v.end()-1);

        return (min_left_front < min_right_front ? min_left_front : min_right_front);
    }

    float get_min_left_laser_range(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const 
    {
        const std::vector<float> v = msg_in -> ranges;
        const auto RANGES_SIZE = v.end() - v.begin();

        return *std::min_element(v.begin() + RANGES_SIZE/4 - RANGES_SIZE/SCALE, v.begin() + RANGES_SIZE/4 + RANGES_SIZE/SCALE);
    }

    float get_min_right_laser_range(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const 
    {
        const std::vector<float> v = msg_in -> ranges;
        const auto RANGES_SIZE = v.end() - v.begin();

        return *std::min_element(v.end() - RANGES_SIZE/4 - RANGES_SIZE/SCALE, v.end() - RANGES_SIZE/4 + RANGES_SIZE/SCALE);
    }



    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scaner_subscription_;
    rclcpp::Publisher<vacuum_cleaner_pkg::msg::Bumper>::SharedPtr bumper_publisher_;

    
};

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<BumperSensor>());
//   rclcpp::shutdown();
//   return 0;
// }