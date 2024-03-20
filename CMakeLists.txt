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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vacuum_cleaner_pkg REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bumper.msg"
  DEPENDENCIES 
)



add_executable(vacuum_cleaner_node src/vacuum_cleaner_node.cpp)
ament_target_dependencies(vacuum_cleaner_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 vacuum_cleaner_pkg)

# add_executable(score_algorithm src/score_algorithm.cpp)
# ament_target_dependencies(score_algorithm rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 vacuum_cleaner_pkg)


# add_executable(bumper_sim src/bumper_sim.cpp)
# ament_target_dependencies(bumper_sim rclcpp sensor_msgs vacuum_cleaner_pkg)

target_include_directories(
  vacuum_cleaner_node PUBLIC
  # score_algorithm PUBLIC
  # bumper_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  vacuum_cleaner_node 
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
