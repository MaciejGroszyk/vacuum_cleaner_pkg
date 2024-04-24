#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <random>
#include <cmath>


#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vacuum_cleaner_pkg/msg/bumper.hpp"



using std::placeholders::_1;

class TurtleBotControl : public rclcpp::Node
{
public:
  TurtleBotControl()
  : Node("turtlebot3_main_cpp")
  {
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::ServicesQoS());

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, std::bind(&TurtleBotControl::odomCallback, this, _1));

    bumper_subscription_ = this->create_subscription<vacuum_cleaner_pkg::msg::Bumper>(
      "/bumper_sensor", 20, std::bind(&TurtleBotControl::bumperCallback, this, _1));

    timer_main = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TurtleBotControl::main, this));

    collison_handling = false;
  } 
  // ~TurtleBotControl()
  // {
  //     // RCLCPP_INFO(this->get_logger(), "Publishing quarter num: '%i'",23);
  // }

  void main()
  {
    spiral_walk();
    //random_walk();
  }


  void spiral_walk()
  {
    if ( (front_detected || left_detected || right_detected) && (not collison_handling))
    {
      generate_new_angle_goal();
      collison_handling = true;
    }
    else if (collison_handling)
    {
      spiral_num = 0;
      collison_handling = collisionHandle();
      move_from_wall_handling = true;
      move_from_wall_time = this->get_clock()->now().seconds();
    }
    else if (move_from_wall_handling)
    {
        const float vel_x = 0.35;
        publish_cmd_vel_function(vel_x, 0.0, 0.0);
        const double time_now =  this->get_clock()->now().seconds();

        if (vel_x*(time_now -move_from_wall_time) > move_from_wall_distance)
          move_from_wall_handling = false;
        
    }
    else if ((not front_detected) && (not collison_handling))
    {
      const int angle_num = get_act_quarter_num();
      // RCLCPP_INFO(this->get_logger(), "Publishing quarter num: '%i'",angle_num);
      // RCLCPP_INFO(this->get_logger(), "Publishing random float: '%s'", is_all_quarters_visited() ? "true" : "false");
      quarters_visited[angle_num] = true;

      if (is_all_quarters_visited())
      {
        spiral_num += 1.0;
        quarters_visited[0] = false;
        quarters_visited[1] = false;
        quarters_visited[2] = false;
        quarters_visited[3] = false;
      }

      const float z_vel = START_YAW_VEL - DECREMENT_YAW_VEL_BY * spiral_num;

      publish_cmd_vel_function(0.35, 0.0, z_vel);
      }   

  }

  bool is_all_quarters_visited() const
  {
    return std::all_of(quarters_visited.begin(), quarters_visited.end(), [](bool v) { return v; });
  }

  void random_walk()
  {
    if (front_detected && (not collison_handling))
    {
      generate_new_angle_goal();
      collison_handling = true;
    }
    else if (collison_handling)
    {
      collison_handling = collisionHandle();
    }
    else if ((not front_detected) && (not collison_handling))
    {
      // RCLCPP_INFO(this->get_logger(), "Publishing cmd vel");
      publish_cmd_vel_function(0.35, 0.0, 0.0);
    }   
  }


  void generate_new_angle_goal()
  {
    // TO DO act yaw not in range
    angle_goal_val =  get_random_value_from_range(-3.14, 3.14);
    // RCLCPP_INFO(this->get_logger(), "Publishing random float: '%f'", angle_goal_val);
  }

  bool collisionHandle()
  {
    float z_vel = (angle_goal_val - act_val_yaw) * K_ANGULAR;
    // RCLCPP_INFO(this->get_logger(), "Publishing z_vel: '%f'", z_vel);
    if (abs(z_vel) < 0.3)
    {
      publish_cmd_vel_function(0.0, 0.0, 0.0);
      return false;
    }
    else
    {
      publish_cmd_vel_function(0.0, 0.0, z_vel);
      return true;
    }
  }


  void bumperCallback(const vacuum_cleaner_pkg::msg::Bumper::SharedPtr msg)
  {
    front_detected = msg -> front_detected;
    left_detected = msg -> left_detected;
    right_detected = msg -> right_detected;
    // RCLCPP_INFO(this->get_logger(), "front_detected: '%s'", front_detected ? "true" : "false");
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    act_val_yaw = yaw;
  }

  void publish_cmd_vel_function(const float x_val, const float y_val, const float z_val)
  {
      auto twist_msg = geometry_msgs::msg::Twist();

      twist_msg.linear.x = x_val;
      twist_msg.linear.y = y_val;
      twist_msg.angular.z = z_val;
      cmd_vel_publisher_->publish(twist_msg);
      //RCLCPP_INFO(this->get_logger(), "Cmd vel published");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<vacuum_cleaner_pkg::msg::Bumper>::SharedPtr bumper_subscription_;
  rclcpp::TimerBase::SharedPtr timer_main;

  float angle_goal_val;
  float act_val_yaw;
  bool front_detected;
  bool right_detected;
  bool left_detected;
  bool collison_handling;
  bool move_from_wall_handling;
  double move_from_wall_time;
  float move_from_wall_distance = 2;
  const float K_ANGULAR = 1.5;



  // spiral_move_config
  std::vector<bool> quarters_visited = std::vector<bool>(4); 
  float spiral_num = 0.0;
  const float DECREMENT_YAW_VEL_BY = 0.2;
  const float START_YAW_VEL = 1.2;



private:

  float get_random_value_from_range(const float min_value, const float max_value) const
  {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<float> dist(min_value, max_value);
    return dist(mt);
  }

  int get_act_quarter_num() const
  {
    //1 | 0
    //-----
    //2 | 3
    const int q_num = int(std::ceil( act_val_yaw/ float(M_PI/2) )) + 1;
    return abs(q_num) > 3 ? 4 : q_num;
  }

};


// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<TurtleBotNode>());
//   rclcpp::shutdown();
//   return 0;
// }