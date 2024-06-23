#include <memory>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"

#include "bumper_sim.cpp"
#include "score_algorithm.cpp"
#include "turtlebot_control.cpp"

int main(int argc, char * argv[])
{
  std::string mode_in = argv[1];
  std::cout << "Mode: " << mode_in << std::endl;
  rclcpp::init(argc, argv);

  auto bumper_sim_node = std::make_shared<BumperSensor>();
  auto turtlebot_control_node = std::make_shared<TurtleBotControl>(mode_in);
  auto score_algorithm_node = std::make_shared<ScoreCoveringAlgorithm>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(bumper_sim_node);
  executor.add_node(score_algorithm_node);
  executor.add_node(turtlebot_control_node);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}