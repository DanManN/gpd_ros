#include <gpd_ros/grasp_detection_server.h>


GraspDetectionServer::GraspDetectionServer(ros::NodeHandle& node)
{
  cloud_camera_ = NULL;

  // set camera viewpoint to default origin
  std::vector<double> camera_position;
  node.param("camera_position", camera_position, std::vector<double> {0.0, 0.0, 0.0});
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
  grasps_pub_ = node.advertise<gpd_ros::GraspConfigList>("clustered_grasps", 10);

  node.getParam("workspace", workspace_);
}

bool GraspDetectionServer::scoreGrasps(gpd_ros::score_grasps::Request& req, gpd_ros::score_grasps::Response& res)
{
  //std::srand(0);
  ROS_INFO("Received scoring service request ...");

  // 1. Initialize cloud camera.
  // cloud_camera_ = NULL;

  if (cloud_camera_ == NULL)
  {
    /* const gpd_ros::CloudSources& cloud_sources = req.cloud_indexed.cloud_sources; */
    const gpd_ros::CloudSources& cloud_sources = req.cloud_samples.cloud_sources;

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


    const auto samples_v = req.cloud_samples.samples;

    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
      << samples_v.size() << " samples");

    Eigen::Matrix3Xd samples(3, samples_v.size());
    for (int i=0; i < samples_v.size(); i++)
    {
      samples.col(i) << samples_v[i].x, samples_v[i].y, samples_v[i].z;
    }

    ROS_INFO_STREAM("Samples: " << samples);
    cloud_camera_->setSamples(samples);

    frame_ = cloud_sources.cloud.header.frame_id;

    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
      << cloud_camera_->getSamples().cols() << " samples");

    // 2. Preprocess the point cloud.
    grasp_detector_->preprocessPointCloud(*cloud_camera_);
  }

  // 3. Detect grasps in the point cloud.
  //std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);
  ROS_INFO_STREAM("Received " << req.grasp_configs.grasps.size() << " grasps to score");
  std::vector<std::unique_ptr<gpd::candidate::Hand>> unscored_grasps = GraspMessages::createHands(req.grasp_configs);
  std::vector<std::unique_ptr<gpd::candidate::HandSet>> hand_set_list(0);
  gpd::candidate::HandSearch::Parameters params = grasp_detector_->getHandSearchParameters();
  gpd::candidate::Antipodal antipodal(params.friction_coeff_, params.min_viable_);
  Eigen::VectorXd angles(0);
  std::vector<int> hand_axes(0);
  Eigen::Array<bool, 1, Eigen::Dynamic> isValid(true);
  //isValid(0) = true;
  for (int i = 0; i < unscored_grasps.size(); i++)
  {
    ROS_INFO_STREAM("Scoring grasp " << i);
    std::unique_ptr<gpd::candidate::HandSet> hand_set(new gpd::candidate::HandSet(
                            params.hand_geometry_,
                            angles,
                            hand_axes,
                            params.num_finger_placements_,
                            params.deepen_hand_,
                            antipodal
                        ));
    Eigen::Vector3d sample = unscored_grasps[i]->getSample();
    hand_set->setSample(sample);
    hand_set->setIsValid(isValid);
    hand_set->hands_.push_back(std::move(unscored_grasps[i]));
    hand_set_list.push_back(std::move(hand_set));
  }
  ROS_INFO_STREAM("Hand set list size: " << hand_set_list.size());

  /**
  // 3. Create grasp descriptors (images).
  double t0_images = omp_get_wtime();
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;
  std::vector<std::unique_ptr<cv::Mat>> images;
  grasp_detector_->image_generator_->createImages(*cloud_camera_, hand_set_list, images, grasps);
  double t_images = omp_get_wtime() - t0_images;
  ROS_INFO_STREAM("Time to create images: " << t_images);

  // 4. Classify the grasp candidates.
  double t0_classify = omp_get_wtime();
  std::vector<float> scores = grasp_detector_->classify_images(images);
  for (int i = 0; i < grasps.size(); i++) {
    grasps[i]->setScore(scores[i]);
  }
  double t_classify = omp_get_wtime() - t0_classify;
  ROS_INFO_STREAM("Time to classify images: " << t_classify);
  **/
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->pruneGraspCandidates(*cloud_camera_, hand_set_list, -1000000);
  //std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = GraspMessages::createHands(req.grasp_configs);
  ROS_INFO_STREAM("After pruning: " << grasps.size() << " grasps left");

  if (grasps.size() > 0)
  {
    // Visualize the detected grasps in rviz.
    if (use_rviz_)
    {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }

    // Publish the detected grasps.
    gpd_ros::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
    res.grasp_configs = selected_grasps_msg;
    ROS_INFO_STREAM("Re-scored " << selected_grasps_msg.grasps.size() << " grasps.");
    return true;
  }

  ROS_WARN("No grasps detected!");
  return false;
}

bool GraspDetectionServer::detectGrasps(gpd_ros::detect_grasps::Request& req, gpd_ros::detect_grasps::Response& res)
{
  //std::srand(0);
  ROS_INFO("Received service request ...");

  // 1. Initialize cloud camera.
  cloud_camera_ = NULL;
  /* const gpd_ros::CloudSources& cloud_sources = req.cloud_indexed.cloud_sources; */
  const gpd_ros::CloudSources& cloud_sources = req.cloud_samples.cloud_sources;

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


  const auto samples_v = req.cloud_samples.samples;

  // Set the indices at which to sample grasp candidates.
  /* std::vector<int> indices(req.cloud_indexed.indices.size()); */
  /* std::vector<int> indices(req.cloud_samples.samples.size()); */
  /* for (int i=0; i < indices.size(); i++) */
  /* { */
  /*   /1* indices[i] = req.cloud_indexed.indices[i].data; *1/ */
  /*   indices[i] = i; */
  /* } */
  /* cloud_camera_->setSampleIndices(indices); */

  /* frame_ = req.cloud_indexed.cloud_sources.cloud.header.frame_id; */

  /* ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and " */
  /*   << req.cloud_indexed.indices.size() << " samples"); */
  // Set the samples at which to sample grasp candidates.

  ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
    << samples_v.size() << " samples");

  Eigen::Matrix3Xd samples(3, samples_v.size());
  for (int i=0; i < samples_v.size(); i++)
  {
    samples.col(i) << samples_v[i].x, samples_v[i].y, samples_v[i].z;
  }

  ROS_INFO_STREAM("Samples: " << samples);
  cloud_camera_->setSamples(samples);

  frame_ = cloud_sources.cloud.header.frame_id;

  ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
  << cloud_camera_->getSamples().cols() << " samples");

  // 2. Preprocess the point cloud.
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // 3. Detect grasps in the point cloud.
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

  if (grasps.size() > 0)
  {
    // Visualize the detected grasps in rviz.
    if (use_rviz_)
    {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }

    // Publish the detected grasps.
    gpd_ros::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
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

  ros::ServiceServer service0 = node.advertiseService("detect_grasps", &GraspDetectionServer::detectGrasps,
                                                     &grasp_detection_server);
  // ros::ServiceServer service1 = node.advertiseService("score_grasps", &GraspDetectionServer::scoreGrasps,
  //                                                   &grasp_detection_server);
  ROS_INFO("Grasp detection service is waiting for a point cloud ...");

  ros::spin();

  return 0;
}
