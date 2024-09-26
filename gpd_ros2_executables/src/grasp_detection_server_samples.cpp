#include <gpd_ros2/grasp_detection_server_samples.h>


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

  node.getParam("workspace", workspace_);
}


bool GraspDetectionServer::detectGrasps(gpd_ros2::detect_grasps_samples::Request& req, gpd_ros2::detect_grasps_samples::Response& res)
{
  ROS_INFO("Received service request ...");

  const gpd_ros2::CloudSamples& msg = req.cloud_samples;
  // 1. Initialize cloud camera.
  cloud_camera_ = NULL;
  const gpd_ros2::CloudSources& cloud_sources = msg.cloud_sources;

  // Set view points.
  Eigen::Matrix3Xd view_points(3, cloud_sources.view_points.size());
  for (int i = 0; i < cloud_sources.view_points.size(); i++)
  {
    view_points.col(i) << cloud_sources.view_points[i].x, cloud_sources.view_points[i].y,
      cloud_sources.view_points[i].z;
  }

  // Set point cloud.
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
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(cloud_sources.cloud, *cloud);

    // TODO: multiple cameras can see the same point
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
    for (int i = 0; i < cloud_sources.camera_source.size(); i++)
    {
      camera_source(cloud_sources.camera_source[i].data, i) = 1;
    }

    cloud_camera_ = new gpd::util::Cloud(cloud, camera_source, view_points);
    std::cout << "view_points:\n" << view_points << "\n";
  }

  ROS_INFO_STREAM("test 0");
  // Set the samples at which to sample grasp candidates.
  Eigen::Matrix3Xd samples(3, msg.samples.size());
  for (int i=0; i < msg.samples.size(); i++)
  {
    samples.col(i) << msg.samples[i].x, msg.samples[i].y, msg.samples[i].z;
  }

  ROS_INFO_STREAM("test 1: " << msg.samples.size());
  ROS_INFO_STREAM("IN: rows " << samples.rows() << " cols " << samples.cols());
  
  if (cloud_camera_ == NULL) {
    ROS_INFO_STREAM("cloud_camera_ NULL");  
  }
  cloud_camera_->setSamples(samples);
  ROS_INFO_STREAM("test 2");
  frame_ = msg.cloud_sources.cloud.header.frame_id;

  ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
    << cloud_camera_->getSamples().cols() << " samples");

  // 2. Preprocess the point cloud.
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  ROS_INFO_STREAM("test 3");
  // 3. Detect grasps in the point cloud.
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

  ROS_INFO_STREAM("test 4" << grasps.size());
  if (grasps.size() > 0)
  {

    // Visualize the detected grasps in rviz.
    if (use_rviz_)
    {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }


    ROS_INFO_STREAM("test 5");
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
  ros::init(argc, argv, "detect_grasps_server");
  ros::NodeHandle node("~");


  GraspDetectionServer grasp_detection_server(node);

  ros::ServiceServer service = node.advertiseService("detect_grasps_samples", &GraspDetectionServer::detectGrasps,
                                                     &grasp_detection_server);
  ROS_INFO("Grasp detection service ready!");

  ros::spin();

  return 0;
}
