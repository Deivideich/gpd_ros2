#ifndef GRASP_MESSAGES_H_
#define GRASP_MESSAGES_H_

#include <Eigen/Dense>
#include <geometry_msgs/msg/vector3.hpp>
#include <gpd_ros2_msgs/msg/grasp_config.hpp>
#include <gpd_ros2_msgs/msg/grasp_config_list.hpp>
#include <gpd/candidate/hand.h>  // Include the correct header for Hand
#include <std_msgs/msg/header.hpp>  // This line is crucial

namespace GraspMessages
{
    geometry_msgs::msg::Vector3 convertToVectorMsg(const Eigen::Vector3d& vec) {
        geometry_msgs::msg::Vector3 msg;
        msg.x = vec.x();
        msg.y = vec.y();
        msg.z = vec.z();
        return msg;
    }

    gpd_ros2_msgs::msg::GraspConfigList createGraspListMsg(
        const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
        const std_msgs::msg::Header& header);
    
    gpd_ros2_msgs::msg::GraspConfig convertToGraspMsg(const gpd::candidate::Hand& hand);
};

#endif /* GRASP_MESSAGES_H_ */
