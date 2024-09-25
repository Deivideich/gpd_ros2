cmake_minimum_required(VERSION 3.8)
project(gpd_ros2)

# Find ament_cmake and other necessary packages
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL 1.9 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

# System dependencies are found with CMake's conventions
find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)

if (NOT GPD_LIB)
  message(FATAL_ERROR "Library GPD not found")
else()
  message(STATUS "Library GPD found in ${GPD_LIB}")
endif()

# Include directories
include_directories(${tf2_eigen_INCLUDE_DIRS} include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Set C++ flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -mtune=intel -msse4.2 -mavx2 -mfma -flto -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17")

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudIndexed.msg"
  "msg/CloudSamples.msg"
  "msg/CloudSources.msg"
  "msg/GraspConfig.msg"
  "msg/GraspConfigList.msg"
  "msg/SamplesMsg.msg"
  "srv/DetectGrasps.srv"
  "srv/DetectGraspsSamples.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs visualization_msgs
)

# Package configuration
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_grasp_messages ${PROJECT_NAME}_grasp_plotter)
ament_export_dependencies(geometry_msgs rclcpp rclpy sensor_msgs std_msgs PCL Eigen3 rosidl_default_runtime)
ament_package()

###########
## Build ##
###########

include_directories(include)

# Declare a C++ library
add_library(${PROJECT_NAME}_grasp_messages src/${PROJECT_NAME}/grasp_messages.cpp)
ament_target_dependencies(${PROJECT_NAME}_grasp_messages
  std_msgs
  visualization_msgs
  sensor_msgs
  tf2_eigen  
  geometry_msgs
  rclcpp
  rclpy
)

add_library(${PROJECT_NAME}_grasp_plotter src/${PROJECT_NAME}/grasp_plotter.cpp)
ament_target_dependencies(${PROJECT_NAME}_grasp_plotter
  std_msgs
  visualization_msgs
  sensor_msgs
  tf2_eigen  
  geometry_msgs
  rclcpp
  rclpy
)

# Declare C++ executables
add_executable(DetectGrasps src/gpd_ros2/grasp_detection_node.cpp)
add_executable(DetectGrasps_server src/gpd_ros2/grasp_detection_server.cpp)
add_executable(DetectGrasps_server_samples src/gpd_ros2/grasp_detection_server_samples.cpp)

# Specify libraries to link against
target_link_libraries(DetectGrasps
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${PCL_LIBRARIES}
                      rclcpp::rclcpp)

target_link_libraries(DetectGrasps_server
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${PCL_LIBRARIES}
                      rclcpp::rclcpp)

target_link_libraries(DetectGrasps_server_samples
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${PCL_LIBRARIES}
                      rclcpp::rclcpp)

target_link_libraries(${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      rclcpp::rclcpp)

target_link_libraries(${PROJECT_NAME}_grasp_messages
                      ${GPD_LIB}
                      rclcpp::rclcpp)
