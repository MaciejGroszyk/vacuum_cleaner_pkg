#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "bumper_sim.cpp"
#include "turtlebot_control.cpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto bumper_sim_node = std::make_shared<BumperSensor>();
  auto turtlebot_control_node = std::make_shared<TurtleBotControl>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(bumper_sim_node);
  executor.add_node(turtlebot_control_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}