cmake_minimum_required(VERSION 3.8)
project(gpd_ros2_executables)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(gpd_ros2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Instead of add_executable, use add_library
add_library(grasp_messages src/grasp_messages.cpp)
# add_library(detection_server src/grasp_detection_server.cpp)

# Include directories for the library
target_include_directories(
  grasp_messages
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# target_include_directories(
#   detection_server
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# target_compile_features(grasp_messages detection_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  grasp_messages
  "std_msgs"
  "visualization_msgs"
  "rclcpp"
  "rclpy"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_eigen"
  "gpd_ros2_msgs"
  "tf2_geometry_msgs"
)

# ament_target_dependencies(
#   detection_server
#   "std_msgs"
#   "visualization_msgs"
#   "rclcpp"
#   "rclpy"
#   "sensor_msgs"
#   "geometry_msgs"
#   "tf2_eigen"
#   "gpd_ros2_msgs"
#   "tf2_geometry_msgs"
# )

install(TARGETS grasp_messages 
#detection_server
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
