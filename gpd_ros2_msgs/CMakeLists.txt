cmake_minimum_required(VERSION 3.8)
project(gpd_ros2_msgs)

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
ament_package()