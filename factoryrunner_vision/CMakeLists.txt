cmake_minimum_required(VERSION 3.8)
project(factoryrunner_vision)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Detection maths with no ROS types, so the gtest runs without a sim.
add_library(depth_obstacles src/depth_obstacles.cpp)
target_include_directories(depth_obstacles PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(depth_obstacles ${OpenCV_LIBS})
target_compile_features(depth_obstacles PUBLIC cxx_std_17)

add_executable(depth_obstacle_detector src/depth_obstacle_detector.cpp)
target_link_libraries(depth_obstacle_detector depth_obstacles)
ament_target_dependencies(depth_obstacle_detector
  rclcpp sensor_msgs geometry_msgs visualization_msgs std_msgs cv_bridge)

install(TARGETS depth_obstacles depth_obstacle_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_obstacles test/test_depth_obstacles.cpp)
  target_link_libraries(test_depth_obstacles depth_obstacles)
endif()

ament_package()
