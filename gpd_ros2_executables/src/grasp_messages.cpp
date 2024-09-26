#include "gpd_ros2_executables/grasp_messages.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // For Eigen and ROS message conversions

gpd_ros2_msgs::msg::GraspConfigList GraspMessages::createGraspListMsg(
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
    const std_msgs::msg::Header& header)
{
    gpd_ros2_msgs::msg::GraspConfigList msg;

    // Loop through each hand and convert it to a GraspConfig message
    for (const auto& hand : hands) {
        msg.grasps.push_back(convertToGraspMsg(*hand));
    }

    msg.header = header;  // Assign the header

    return msg;
}

gpd_ros2_msgs::msg::GraspConfig GraspMessages::convertToGraspMsg(const gpd::candidate::Hand& hand)
{
    gpd_ros2_msgs::msg::GraspConfig msg;

    // Manually convert Eigen::Vector3d to geometry_msgs::msg::Point for position
    const Eigen::Vector3d position_vector = hand.getPosition();
    msg.position.x = position_vector.x();
    msg.position.y = position_vector.y();
    msg.position.z = position_vector.z();

    // Manually convert Eigen::Vector3d to geometry_msgs::msg::Vector3 for approach
    const Eigen::Vector3d approach_vector = hand.getApproach();
    msg.approach.x = approach_vector.x();
    msg.approach.y = approach_vector.y();
    msg.approach.z = approach_vector.z();

    // Similarly for binormal and axis
    const Eigen::Vector3d binormal_vector = hand.getBinormal();
    msg.binormal.x = binormal_vector.x();
    msg.binormal.y = binormal_vector.y();
    msg.binormal.z = binormal_vector.z();

    const Eigen::Vector3d axis_vector = hand.getAxis();
    msg.axis.x = axis_vector.x();
    msg.axis.y = axis_vector.y();
    msg.axis.z = axis_vector.z();

    // Assuming width and score are of type std_msgs::msg::Float32
    msg.width.data = hand.getGraspWidth();
    msg.score.data = hand.getScore();

    // Convert sample to Point
    const Eigen::Vector3d sample_vector = hand.getSample();
    msg.sample.x = sample_vector.x();
    msg.sample.y = sample_vector.y();
    msg.sample.z = sample_vector.z();

    return msg;
}