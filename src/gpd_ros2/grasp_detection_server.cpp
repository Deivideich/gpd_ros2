#include <gpd_ros2/grasp_detection_server.h>


GraspDetectionServer::GraspDetectionServer(ros::NodeHandle& node)
{
  cloud_camera_ = NULL;

  // set camera viewpoint to default origin
  std::vector<double> camera_position;
  std::vector<double> camera_position_default = {0.0, 0.0, 0.0};
  node.param<std::vector<double>>("camera_position", camera_position, camera_position_default);
  view_point_ << camera_position[0], camera_position[1], camera_position[2];

  std::string cfg_file;
  node.param("config_file", cfg_file, std::string(""));
  grasp_detector_ = new gpd::GraspDetector(cfg_file);

  std::string rviz_topic;
  node.param("rviz_topic", rviz_topic, std::string(""));

  if (!rviz_topic.empty())
  {
    rviz_plotter_ = new GraspPlotter(node, grasp_detector_->getHandSearchParameters().hand_geometry_);
    use_rviz_ = true;
  }
  else
  {
    use_rviz_ = false;
  }

  // Advertise ROS topic for detected grasps.
  grasps_pub_ = node.advertise<gpd_ros2::GraspConfigList>("clustered_grasps", 10);
  pc_pub_1_ = node.advertise<sensor_msgs::PointCloud2>("/test_pc_Server", 10);

  node.getParam("workspace", workspace_);
}


bool GraspDetectionServer::DetectGrasps(gpd_ros2::DetectGrasps::Request& req, gpd_ros2::DetectGrasps::Response& res)
{
  ROS_INFO("Received service request ...");

  // 1. Initialize cloud camera.
  cloud_camera_ = NULL;
  const gpd_ros2::CloudSources& cloud_sources = req.cloud_indexed.cloud_sources;

  // Set view points.
  Eigen::Matrix3Xd view_points(3, cloud_sources.view_points.size());
  for (int i = 0; i < cloud_sources.view_points.size(); i++)
  {
    view_points.col(i) << cloud_sources.view_points[i].x, cloud_sources.view_points[i].y,
      cloud_sources.view_points[i].z;
  }

  // Set point cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  if (cloud_sources.cloud.fields.size() == 6 && cloud_sources.cloud.fields[3].name == "normal_x"
    && cloud_sources.cloud.fields[4].name == "normal_y" && cloud_sources.cloud.fields[5].name == "normal_z")
  {
    PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
    pcl::fromROSMsg(cloud_sources.cloud, *cloud);

    // TODO: multiple cameras can see the same point
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
    for (int i = 0; i < cloud_sources.camera_source.size(); i++)
    {
      camera_source(cloud_sources.camera_source[i].data, i) = 1;
    }

    cloud_camera_ = new gpd::util::Cloud(cloud, camera_source, view_points);
  }
  else
  {
    pcl::fromROSMsg(cloud_sources.cloud, *cloud_xyz);
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::copyPointCloud(*cloud_xyz, *cloud);

    // TODO: multiple cameras can see the same point
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
    for (int i = 0; i < cloud_sources.camera_source.size(); i++)
    {
      camera_source(cloud_sources.camera_source[i].data, i) = 1;
    }

    cloud_camera_ = new gpd::util::Cloud(cloud, camera_source, view_points);
    std::cout << "view_points:\n" << view_points << "\n";
  }

  // Set the indices at which to sample grasp candidates.
  std::vector<int> indices(req.cloud_indexed.indices.size());
  for (int i=0; i < indices.size(); i++)
  {
    indices[i] = req.cloud_indexed.indices[i].data;
  }
  cloud_camera_->setSampleIndices(indices);
  
  // TEST
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 tmp;
  ROS_INFO_STREAM("Points in old: " <<cloud_xyz->points.size());
  ROS_INFO_STREAM("Inliers: " <<indices.size());

  pcl::copyPointCloud(*cloud_xyz, indices, *tmpPC);
  pcl::toROSMsg(*tmpPC, tmp);
  tmp.header.frame_id = cloud_sources.cloud.header.frame_id;
  tmp.header.stamp = cloud_sources.cloud.header.stamp;
  pc_pub_1_.publish(tmp);
  
  // END
  frame_ = cloud_sources.cloud.header.frame_id;

  ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
    << req.cloud_indexed.indices.size() << " samples");

  // 2. Preprocess the point cloud.
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // 3. Detect grasps in the point cloud.
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->DetectGrasps(*cloud_camera_);

  if (grasps.size() > 0)
  {
    // Visualize the detected grasps in rviz.
    if (use_rviz_)
    {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }

    // Publish the detected grasps.
    gpd_ros2::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
    res.grasp_configs = selected_grasps_msg;
    ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
    return true;
  }

  ROS_WARN("No grasps detected!");
  return false;
}

int main(int argc, char** argv)
{
  // seed the random number generator
  std::srand(std::time(0));

  // initialize ROS
  ros::init(argc, argv, "DetectGrasps_server");
  ros::NodeHandle node("~");

  GraspDetectionServer grasp_detection_server(node);

  ros::ServiceServer service = node.advertiseService("DetectGrasps", &GraspDetectionServer::DetectGrasps,
                                                     &grasp_detection_server);
  ROS_INFO("Grasp detection service is waiting for a point cloud ...");

  ros::spin();

  return 0;
}
