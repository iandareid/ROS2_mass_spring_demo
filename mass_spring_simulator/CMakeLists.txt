cmake_minimum_required(VERSION 3.8)
project(mass_spring_simulator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mass_spring_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

##################
## INCLUDE DIRS ##
##################
include_directories(
  include
)

#################
## EXTRA FILES ##
#################
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

####################
## SIM EXECUTABLE ##
####################
add_executable(simulator
  src/simulator_node.cpp
  src/simulator.cpp
)
ament_target_dependencies(simulator mass_spring_msgs rclcpp std_msgs visualization_msgs)
install(TARGETS
  simulator
  DESTINATION lib/${PROJECT_NAME}
)

add_library(simulator_lib include/simulator.hpp src/simulator.cpp)
target_link_libraries(simulator simulator_lib)
ament_target_dependencies(simulator_lib rclcpp rclcpp std_msgs visualization_msgs mass_spring_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mass_spring_test test/mass_test.cpp)
  ament_target_dependencies(mass_spring_test mass_spring_msgs rclcpp std_msgs visualization_msgs)
  target_include_directories(mass_spring_test PUBLIC
    include
    include>
  )
  target_link_libraries(mass_spring_test simulator_lib) 
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
