cmake_minimum_required(VERSION 3.5)
project(vacuum_cleaner_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vacuum_cleaner_pkg REQUIRED)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bumper.msg"
  DEPENDENCIES std_msgs
)

add_executable(vacuum_cleaner_node src/vacuum_cleaner_node.cpp)
ament_target_dependencies(vacuum_cleaner_node rclcpp nlohmann_json sensor_msgs geometry_msgs nav_msgs ament_index_cpp std_msgs tf2 OpenCV vacuum_cleaner_pkg)


add_executable(my_fun_node src/my_fun.cpp)
ament_target_dependencies(my_fun_node rclcpp sensor_msgs nlohmann_json geometry_msgs nav_msgs ament_index_cpp std_msgs tf2 OpenCV vacuum_cleaner_pkg)

add_executable(keypad_node 
    src2/robot_control_keypad.cpp
    src2/robot_handler/RobotController.cpp
    src2/robot_handler/RobotController.h
    )
ament_target_dependencies(keypad_node rclcpp geometry_msgs ament_index_cpp vacuum_cleaner_pkg)


add_executable(main_node 
  src2/main.cpp
  src2/robot_handler/LaserHandler.cpp
  src2/robot_handler/LaserHandler.h
  src2/robot_handler/OdomHandler.cpp
  src2/robot_handler/OdomHandler.h
  src2/robot_handler/RobotController.cpp
  src2/robot_handler/RobotController.h
  src2/covering_algorithms/CoveringAlgorithm.cpp
  src2/covering_algorithms/CoveringAlgorithm.h
  src2/covering_algorithms/CoveringAlgorithmHandler.cpp
  src2/covering_algorithms/CoveringAlgorithmHandler.h
  src2/covering_algorithms/RandomWalk.cpp
  src2/covering_algorithms/RandomWalk.h
  src2/covering_algorithms/SpiralWalk.cpp
  src2/covering_algorithms/SpiralWalk.h
  )
ament_target_dependencies(main_node rclcpp nlohmann_json sensor_msgs geometry_msgs nav_msgs ament_index_cpp std_msgs tf2 OpenCV vacuum_cleaner_pkg)


# add_executable(score_algorithm src/score_algorithm.cpp)
# ament_target_dependencies(score_algorithm rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 vacuum_cleaner_pkg)


# add_executable(bumper_sim src/bumper_sim.cpp)
# ament_target_dependencies(bumper_sim rclcpp sensor_msgs vacuum_cleaner_pkg)

target_include_directories(
  vacuum_cleaner_node PUBLIC
  keypad_node PUBLIC
  my_fun_node PUBLIC
  main_node PUBLIC
  # score_algorithm PUBLIC
  # bumper_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  vacuum_cleaner_node 
  my_fun_node
  keypad_node
  main_node
  # score_algorithm
  # bumper_sim
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
