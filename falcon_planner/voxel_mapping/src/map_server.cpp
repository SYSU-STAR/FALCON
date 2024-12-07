#include "voxel_mapping/map_server.h"

namespace voxel_mapping {
MapServer::MapServer(ros::NodeHandle &nh) {
  tsdf_.reset(new TSDF());
  esdf_.reset(new ESDF());
  occupancy_grid_.reset(new OccupancyGrid());
  transformer_.reset(new Transformer(nh));

  /* ------------------------ Map general configuration ----------------------- */
  MapConfig &map_config = tsdf_->map_config_;
  nh.param("/voxel_mapping/resolutionf_fine", map_config.resolution_fine_, 0.1);
  nh.param("/voxel_mapping/resolution_coarse", map_config.resolution_coarse_, 0.2);

  vector<string> axis = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    nh.param("/map_config/map_size/map_min_" + axis[i], map_config.map_min_[i], 0.0);
    nh.param("/map_config/map_size/map_max_" + axis[i], map_config.map_max_[i], 0.0);
    nh.param("/map_config/map_size/box_min_" + axis[i], map_config.box_min_[i],
             map_config.map_min_[i]);
    nh.param("/map_config/map_size/box_max_" + axis[i], map_config.box_max_[i],
             map_config.map_max_[i]);
    nh.param("/map_config/map_size/vbox_min_" + axis[i], map_config.vbox_min_[i],
             map_config.map_min_[i]);
    nh.param("/map_config/map_size/vbox_max_" + axis[i], map_config.vbox_max_[i],
             map_config.map_max_[i]);
  }

  // Check map size configuration
  for (int i = 0; i < 3; ++i) {
    CHECK_LT(map_config.map_min_[i], map_config.map_max_[i]);
    CHECK_LT(map_config.box_min_[i], map_config.box_max_[i]);
    CHECK_LT(map_config.vbox_min_[i], map_config.vbox_max_[i]);
    CHECK_LE(map_config.map_min_[i], map_config.box_min_[i]);
    CHECK_GE(map_config.map_max_[i], map_config.box_max_[i]);
    CHECK_LE(map_config.map_min_[i], map_config.vbox_min_[i]);
    CHECK_GE(map_config.map_max_[i], map_config.vbox_max_[i]);
  }

  double box_volume = (map_config.box_max_ - map_config.box_min_).prod();
  if (box_volume < 4000.0) {
    map_config.resolution_ = map_config.resolution_fine_;
  } else {
    map_config.resolution_ = map_config.resolution_coarse_;
  }
  map_config.resolution_inv_ = 1 / map_config.resolution_;

  map_config.map_size_ = map_config.map_max_ - map_config.map_min_;
  for (int i = 0; i < 3; ++i)
    map_config.map_size_idx_(i) = ceil(map_config.map_size_(i) / map_config.resolution_);
  tsdf_->positionToIndex(map_config.box_min_, map_config.box_min_idx_);
  tsdf_->positionToIndex(map_config.box_max_, map_config.box_max_idx_);
  tsdf_->positionToIndex(map_config.vbox_min_, map_config.vbox_min_idx_);
  tsdf_->positionToIndex(map_config.vbox_max_, map_config.vbox_max_idx_);

  // Use same general configuration
  esdf_->map_config_ = map_config;
  occupancy_grid_->map_config_ = map_config;

  /* ----------------------- Map specific configurations ---------------------- */
  // Map server configuration
  nh.param("/uav_model/sensing_parameters/camera_intrinsics/fx", config_.fx_, 0.0);
  nh.param("/uav_model/sensing_parameters/camera_intrinsics/fy", config_.fy_, 0.0);
  nh.param("/uav_model/sensing_parameters/camera_intrinsics/cx", config_.cx_, 0.0);
  nh.param("/uav_model/sensing_parameters/camera_intrinsics/cy", config_.cy_, 0.0);
  nh.param("/voxel_mapping/depth_filter_margin", config_.depth_filter_margin_, 0);
  nh.param("/voxel_mapping/depth_scaling_factor", config_.depth_scaling_factor_, 1000.0);
  config_.depth_scaling_factor_inv_ = 1.0 / config_.depth_scaling_factor_;
  nh.param("/voxel_mapping/depth_decimation_filter", config_.depth_decimation_filter_, false);
  nh.param("/voxel_mapping/skip_pixel", config_.skip_pixel_, 0);
  nh.param("/voxel_mapping/concurrent_depth_input_max", config_.concurrent_depth_input_max_, 1);
  nh.param("/voxel_mapping/concurrent_pointcloud_input_max",
           config_.concurrent_pointcloud_input_max_, 1);
  nh.param("/voxel_mapping/publish_tsdf", config_.publish_tsdf_, false);
  nh.param("/voxel_mapping/publish_esdf", config_.publish_esdf_, false);
  nh.param("/voxel_mapping/publish_occupancy_grid", config_.publish_occupancy_grid_, false);
  nh.param("/voxel_mapping/publish_tsdf_slice", config_.publish_tsdf_slice_, false);
  nh.param("/voxel_mapping/publish_esdf_slice", config_.publish_esdf_slice_, false);
  nh.param("/voxel_mapping/publish_occupancy_grid_slice", config_.publish_occupancy_grid_slice_,
           false);
  nh.param("/voxel_mapping/publish_tsdf_period", config_.publish_tsdf_period_, 1.0);
  nh.param("/voxel_mapping/publish_esdf_period", config_.publish_esdf_period_, 1.0);
  nh.param("/voxel_mapping/publish_occupancy_grid_period", config_.publish_occupancy_grid_period_,
           1.0);
  nh.param("/voxel_mapping/tsdf_slice_height", config_.tsdf_slice_height_, -1.0);
  nh.param("/voxel_mapping/tsdf_slice_visualization_height",
           config_.tsdf_slice_visualization_height_, -1.0);
  nh.param("/voxel_mapping/esdf_slice_height", config_.esdf_slice_height_, -1.0);
  nh.param("/voxel_mapping/esdf_slice_visualization_height",
           config_.esdf_slice_visualization_height_, -1.0);
  nh.param("/voxel_mapping/occupancy_grid_slice_height", config_.occupancy_grid_slice_height_,
           -1.0);
  nh.param("/voxel_mapping/occupancy_grid_slice_visualization_height",
           config_.occupancy_grid_slice_visualization_height_, -1.0);

  nh.param("/voxel_mapping/verbose", config_.verbose_, false);
  nh.param("/voxel_mapping/verbose_time", config_.verbose_time_, false);

  config_.world_frame_ = transformer_->getWorldFrame();
  config_.sensor_frame_ = transformer_->getSensorFrame();

  if (config_.verbose_)
    config_.print();

  string mode_string;
  nh.param("/voxel_mapping/mode", mode_string, string(""));
  if (mode_string == "airsim")
    config_.mode_ = MapServer::Config::Mode::AIRSIM;
  else if (mode_string == "uav_simulator")
    config_.mode_ = MapServer::Config::Mode::UAV_SIMULATOR;
  else if (mode_string == "real")
    config_.mode_ = MapServer::Config::Mode::REAL;
  else
    CHECK(false) << "Unknown mode: " << mode_string;

  // TSDF configuration
  TSDF::Config &tsdf_config = tsdf_->config_;

  double tsdf_truncated_dist_scale = 1.0;
  nh.param("/voxel_mapping/tsdf/tsdf_truncated_distance_scale", tsdf_truncated_dist_scale, 1.0);
  tsdf_config.truncated_dist_ = map_config.resolution_ * tsdf_truncated_dist_scale;

  nh.param("/voxel_mapping/tsdf/raycast_min", tsdf_config.raycast_min_, 0.0);
  nh.param("/voxel_mapping/tsdf/raycast_max", tsdf_config.raycast_max_, 5.0);
  nh.param("/voxel_mapping/tsdf/epsilon", tsdf_config.epsilon_, 1e-4);

  if (config_.mode_ == MapServer::Config::Mode::AIRSIM)
    tsdf_config.depth_axis_ = TSDF::Config::DepthAxis::Y;
  else
    tsdf_config.depth_axis_ = TSDF::Config::DepthAxis::Z;

  // ESDF configuration

  // Occupancy grid configuration
  OccupancyGrid::Config &occupancy_grid_config = occupancy_grid_->config_;
  double sdf_cutoff_distance_scale = 1.0;
  nh.param("/voxel_mapping/occupancy_grid/sdf_cutoff_distance_scale", sdf_cutoff_distance_scale,
           1.0);
  occupancy_grid_config.sdf_cutoff_dist_ = map_config.resolution_ * sdf_cutoff_distance_scale;

  string occupancy_grid_mode;
  nh.param("/voxel_mapping/occupancy_grid/occupancy_grid_mode", occupancy_grid_mode,
           string("GEN_TSDF"));
  if (occupancy_grid_mode == "GEN_TSDF")
    occupancy_grid_config.mode_ = OccupancyGrid::Config::MODE::GEN_TSDF;
  else if (occupancy_grid_mode == "GEN_PCL")
    occupancy_grid_config.mode_ = OccupancyGrid::Config::MODE::GEN_PCL;
  else
    CHECK(false) << "Unknown occupancy grid mode: " << occupancy_grid_mode;

  /* ----------------------------- Initialization ----------------------------- */
  // Initialize TSDF
  tsdf_->initMapData();
  tsdf_->initRaycaster();
  tsdf_->setESDF(esdf_);
  tsdf_->setOccupancyGrid(occupancy_grid_);

  // Initialize ESDF
  esdf_->initMapData();
  esdf_->initRaycaster();
  esdf_->setTSDF(tsdf_);
  esdf_->setOccupancyGrid(occupancy_grid_);

  // Initialize occupancy grid
  occupancy_grid_->initMapData();
  occupancy_grid_->initRaycaster();
  occupancy_grid_->setTSDF(tsdf_);

  // Initialize publishers
  tsdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/tsdf", 10);
  tsdf_slice_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/tsdf_slice", 10);
  esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/esdf", 10);
  esdf_slice_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/esdf_slice", 10);
  occupancy_grid_occupied_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/occupancy_grid_occupied", 10);
  occupancy_grid_free_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/occupancy_grid_free", 10);
  occupancy_grid_unknown_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/occupancy_grid_unknown", 10);
  occupancy_grid_slice_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/occupancy_grid_slice", 10);
  interpolated_pose_pub_ = nh.advertise<nav_msgs::Odometry>("/voxel_mapping/interpolated_pose", 10);
  depth_pointcloud_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/depth_pointcloud", 10);
  map_coverage_pub_ = nh.advertise<std_msgs::Float32>("/voxel_mapping/map_coverage", 10);
  debug_visualization_pub_ =
      nh.advertise<visualization_msgs::Marker>("/voxel_mapping/debug_visualization", 10);

  // Initialize callbacks
  depth_sub_ = nh.subscribe("/voxel_mapping/depth_image", 1, &MapServer::depthCallback, this);
  pointcloud_sub_ =
      nh.subscribe("/voxel_mapping/pointcloud", 1, &MapServer::pointcloudCallback, this);
  global_map_sub_ =
      nh.subscribe("/voxel_mapping/global_map", 1, &MapServer::globalMapCallback, this);

  // Initialize service
  save_map_srv_ =
      nh.advertiseService("/voxel_mapping/save_map_pcd", &MapServer::saveMapPCDCallback, this);

  // Initialize timer
  publish_map_timer_ =
      nh.createTimer(ros::Duration(0.5), &MapServer::publishMapTimerCallback, this);

  if (config_.verbose_) {
    ROS_INFO("[MapServer] Voxel mapping server initialized with parameters: ");
    ROS_INFO("  - Resolution: %.2f", map_config.resolution_);
    ROS_INFO("  - Map size: %.2f x %.2f x %.2f", map_config.map_size_(0), map_config.map_size_(1),
             map_config.map_size_(2));
    ROS_INFO("  - Map size index: %d x %d x %d", map_config.map_size_idx_(0),
             map_config.map_size_idx_(1), map_config.map_size_idx_(2));
    ROS_INFO("  - Box min: %.2f , %.2f , %.2f", map_config.box_min_(0), map_config.box_min_(1),
             map_config.box_min_(2));
    ROS_INFO("  - Box max: %.2f , %.2f , %.2f", map_config.box_max_(0), map_config.box_max_(1),
             map_config.box_max_(2));
    ROS_INFO("  - Box min index: %d , %d , %d", map_config.box_min_idx_(0),
             map_config.box_min_idx_(1), map_config.box_min_idx_(2));
    ROS_INFO("  - Box max index: %d , %d , %d", map_config.box_max_idx_(0),
             map_config.box_max_idx_(1), map_config.box_max_idx_(2));
    ROS_INFO("  - Visualizing Box min: %.2f , %.2f , %.2f", map_config.vbox_min_(0),
             map_config.vbox_min_(1), map_config.vbox_min_(2));
    ROS_INFO("  - Visualizing Box max: %.2f , %.2f , %.2f", map_config.vbox_max_(0),
             map_config.vbox_max_(1), map_config.vbox_max_(2));
    ROS_INFO("  - Visualizing Box min index: %d , %d , %d", map_config.vbox_min_idx_(0),
             map_config.vbox_min_idx_(1), map_config.vbox_min_idx_(2));
    ROS_INFO("  - Visualizing Box max index: %d , %d , %d", map_config.vbox_max_idx_(0),
             map_config.vbox_max_idx_(1), map_config.vbox_max_idx_(2));
    ROS_INFO("  - TSDF truncated distance: %.2f", tsdf_config.truncated_dist_);
  }

  pos_min_ = Position::Zero();
  pos_max_ = Position::Zero();
  map_update_counter_ = 0;
}

void MapServer::depthCallback(const sensor_msgs::ImageConstPtr &image_msg) {
  if (config_.verbose_) {
    ROS_INFO("[MapServer] Received depth image with stamp: %f", image_msg->header.stamp.toSec());
  }

  if (this->map_update_counter_ > config_.concurrent_depth_input_max_)
    return;

  TicToc tic1;
  cv::Mat depth_image;
  PointCloudType pointcloud;
  Transformation sensor_pose;
  ros::Time img_stamp;

  image_queue_.push(image_msg);

  while (getNextImageFromQueue(image_queue_, depth_image, sensor_pose, img_stamp)) {
    if (config_.depth_decimation_filter_)
      // Decimation filter for real-world RealSense D435i depth noise reduction
      depthToPointcloudDecimation(depth_image, sensor_pose, pointcloud, img_stamp);
    else
      depthToPointcloud(depth_image, sensor_pose, pointcloud, img_stamp);

    for (int i = 0; i < 3; i++) {
      pos_min_(i) = min(pos_min_(i), sensor_pose.getPosition()(i));
      pos_max_(i) = max(pos_max_(i), sensor_pose.getPosition()(i));
    }

    if (config_.verbose_) {
      ROS_INFO("[MapServer] pos_min: %.2f, %.2f, %.2f", pos_min_(0), pos_min_(1), pos_min_(2));
      ROS_INFO("[MapServer] pos_max: %.2f, %.2f, %.2f", pos_max_(0), pos_max_(1), pos_max_(2));
    }

    publishDebugVisualizationBBox(pos_min_, pos_max_);
    publishInterpolatedPose(sensor_pose, img_stamp);

    TicToc tic2;
    std::thread tsdf_thread([this, pointcloud]() {
      this->map_update_counter_++;
      TicToc tic3;
      std::lock_guard<std::mutex> lock(this->map_mutex_);
      this->tsdf_->inputPointCloud(pointcloud);
      if (config_.verbose_time_) {
        ROS_INFO("[MapServer] inputPointCloud thread time: %fs", tic3.toc());
      }
      // Warn if inputPointCloud process frequency lower than 10Hz
      ROS_WARN_COND(tic3.toc() > 0.1, "[MapServer] inputPointCloud thread time too long: %fs",
                    tic3.toc());
      this->map_update_counter_--;
    });
    tsdf_thread.detach();

    if (config_.verbose_time_) {
      ROS_INFO("[MapServer] Depth pointcloud size: %d", (int)pointcloud.size());
      ROS_INFO("[MapServer] Depth pointcloud process time: %fs", tic2.toc());
    }
  }

  if (config_.verbose_time_) {
    ROS_BLUE_STREAM("[MapServer] Depth callback time: " << tic1.toc());
  }
}

void MapServer::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg) {
  if (config_.verbose_) {
    ROS_INFO("[MapServer] Received pointcloud with stamp: %f",
             pointcloud_msg->header.stamp.toSec());
  }

  if (this->map_update_counter_ > config_.concurrent_depth_input_max_)
    return;

  TicToc tic1;
  PointCloudType pointcloud;
  Transformation sensor_pose;
  ros::Time pcl_stamp;

  pointcloud_queue_.push(pointcloud_msg);

  while (getNextPointcloudFromQueue(pointcloud_queue_, pointcloud, sensor_pose, pcl_stamp)) {
    for (int i = 0; i < 3; i++) {
      pos_min_(i) = min(pos_min_(i), sensor_pose.getPosition()(i));
      pos_max_(i) = max(pos_max_(i), sensor_pose.getPosition()(i));
    }

    if (config_.verbose_time_) {
      ROS_INFO("[MapServer] pos_min: %.2f, %.2f, %.2f", pos_min_(0), pos_min_(1), pos_min_(2));
      ROS_INFO("[MapServer] pos_max: %.2f, %.2f, %.2f", pos_max_(0), pos_max_(1), pos_max_(2));
    }
    publishDebugVisualizationBBox(pos_min_, pos_max_);
    publishInterpolatedPose(sensor_pose, pcl_stamp);

    TicToc tic2;
    std::thread tsdf_thread([this, pointcloud]() {
      this->map_update_counter_++;
      TicToc tic3;
      std::lock_guard<std::mutex> lock(this->map_mutex_);
      this->tsdf_->inputPointCloud(pointcloud);
      if (config_.verbose_time_) {
        ROS_INFO("[MapServer] inputPointCloud thread time: %fs", tic3.toc());
      }
      this->map_update_counter_--;
    });
    tsdf_thread.detach();

    if (config_.verbose_time_) {
      ROS_INFO("[MapServer] Pointcloud size: %d", (int)pointcloud.size());
      ROS_INFO("[MapServer] Pointcloud process time: %fs", tic2.toc());
    }
  }

  if (config_.verbose_time_) {
    ROS_INFO("[MapServer] Pointcloud callback time: %fs", tic1.toc());
  }
}

void MapServer::globalMapCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  static bool load_once = true;
  if (load_once) {
    PointCloudType::Ptr cloud(new PointCloudType);
    pcl::fromROSMsg(*msg, *cloud);
    loadMapFromPCL(cloud);
    load_once = false;
  }
}

bool MapServer::saveMapPCDCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("[MapServer] Saving map to PCD file");

  string filename = "/home/eason/map.pcd";
  saveMapToPCD(filename);

  return true;
}

void MapServer::testESDFCallback(const geometry_msgs::PoseStamped &msg) {
  Position pos = Position(msg.pose.position.x, msg.pose.position.y, 1.0);
  double dist;
  Eigen::Vector3d grad;

  esdf_->getDistanceAndGradient(pos, dist, grad);

  ROS_INFO("[MapServer] Test ESDF at position: %.2f, %.2f, %.2f, distance: %.2f, gradient: %.2f, "
           "%.2f, %.2f",
           pos(0), pos(1), pos(2), dist, grad(0), grad(1), grad(2));
}

void MapServer::publishMapTimerCallback(const ros::TimerEvent &event) {
  if (config_.publish_tsdf_)
    publishTSDF();

  if (config_.publish_tsdf_slice_)
    publishTSDFSlice();

  if (config_.publish_esdf_)
    publishESDF();

  if (config_.publish_esdf_slice_)
    publishESDFSlice();

  if (config_.publish_occupancy_grid_) {
    if (getResolution() > 0.1)
      publishOccupancyGridHighResolution();
    else
      publishOccupancyGrid();
  }

  if (config_.publish_occupancy_grid_slice_)
    publishOccupancyGridSlice();

  publishMapCoverage();
}

void MapServer::publishTSDF() {
  if (tsdf_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing TSDF...");

  PointCloudIntensityType pointcloud;
  PointIntensityType point;
  const FloatingPoint min_dist = -tsdf_->config_.truncated_dist_;
  const FloatingPoint max_dist = tsdf_->config_.truncated_dist_;

  for (int x = tsdf_->map_config_.vbox_min_idx_[0]; x < tsdf_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = tsdf_->map_config_.vbox_min_idx_[1]; y < tsdf_->map_config_.vbox_max_idx_[1];
         ++y) {
      for (int z = tsdf_->map_config_.vbox_min_idx_[2]; z < tsdf_->map_config_.vbox_max_idx_[2];
           ++z) {
        VoxelIndex idx(x, y, z);
        Position pos = tsdf_->indexToPosition(idx);

        // unknown or cleared voxel
        if (tsdf_->getVoxel(idx).weight < tsdf_->config_.epsilon_)
          continue;

        point.x = pos(0);
        point.y = pos(1);
        point.z = pos(2);
        point.intensity = (tsdf_->getVoxel(idx).value - min_dist) / (max_dist - min_dist);
        pointcloud.points.push_back(point);
      }
    }
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  tsdf_pub_.publish(pointcloud_msg);
}

void MapServer::publishTSDFSlice() {
  if (tsdf_slice_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing TSDF slice...");

  PointCloudIntensityType pointcloud;
  PointIntensityType point;
  const FloatingPoint min_dist = -tsdf_->config_.truncated_dist_;
  const FloatingPoint max_dist = tsdf_->config_.truncated_dist_;

  for (int x = tsdf_->map_config_.vbox_min_idx_[0]; x < tsdf_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = tsdf_->map_config_.vbox_min_idx_[1]; y < tsdf_->map_config_.vbox_max_idx_[1];
         ++y) {
      VoxelIndex idx(x, y, 0);
      Position pos = tsdf_->indexToPosition(idx);
      pos.z() = config_.tsdf_slice_height_;
      tsdf_->positionToIndex(pos, idx);

      // unknown or cleared voxel
      if (tsdf_->getVoxel(idx).weight < tsdf_->config_.epsilon_)
        continue;

      point.x = pos(0);
      point.y = pos(1);
      point.z = config_.tsdf_slice_visualization_height_;
      point.intensity = (tsdf_->getVoxel(idx).value - min_dist) / (max_dist - min_dist);
      pointcloud.points.push_back(point);
    }
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  tsdf_slice_pub_.publish(pointcloud_msg);
}

void MapServer::publishESDF() {
  if (esdf_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing ESDF...");

  PointCloudIntensityType pointcloud;
  PointIntensityType point;

  for (int x = esdf_->map_config_.vbox_min_idx_[0]; x < esdf_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = esdf_->map_config_.vbox_min_idx_[1]; y < esdf_->map_config_.vbox_max_idx_[1];
         ++y) {
      for (int z = esdf_->map_config_.vbox_min_idx_[2]; z < esdf_->map_config_.vbox_max_idx_[2];
           ++z) {
        VoxelIndex idx(x, y, z);
        Position pos = esdf_->indexToPosition(idx);

        // unknown or cleared voxel
        if (esdf_->getVoxel(idx).value < 0)
          continue;

        point.x = pos(0);
        point.y = pos(1);
        point.z = pos(2);
        point.intensity = esdf_->getVoxel(idx).value;
        pointcloud.points.push_back(point);
      }
    }
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  esdf_pub_.publish(pointcloud_msg);
}

void MapServer::publishESDFSlice() {
  if (esdf_slice_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing ESDF slice...");

  PointCloudIntensityType pointcloud;
  PointIntensityType point;

  for (int x = esdf_->map_config_.vbox_min_idx_[0]; x < esdf_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = esdf_->map_config_.vbox_min_idx_[1]; y < esdf_->map_config_.vbox_max_idx_[1];
         ++y) {
      VoxelIndex idx(x, y, 0);
      Position pos = esdf_->indexToPosition(idx);
      pos.z() = config_.esdf_slice_height_;
      esdf_->positionToIndex(pos, idx);

      if (occupancy_grid_->getVoxel(idx).value == OccupancyType::UNKNOWN)
        continue;

      point.x = pos(0);
      point.y = pos(1);
      point.z = config_.esdf_slice_visualization_height_;
      point.intensity = esdf_->getVoxel(idx).value;
      pointcloud.points.push_back(point);
    }
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  esdf_slice_pub_.publish(pointcloud_msg);
}

void MapServer::publishOccupancyGrid() {
  if (occupancy_grid_occupied_pub_.getNumSubscribers() == 0 &&
      occupancy_grid_free_pub_.getNumSubscribers() == 0 &&
      occupancy_grid_unknown_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO_THROTTLE(1.0, "[MapServer] Publishing occupancy grid...");

  PointCloudType pointcloud_occupied, pointcloud_free, pointcloud_unknown;
  PointType point;

  for (int x = tsdf_->map_config_.vbox_min_idx_[0]; x < tsdf_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = tsdf_->map_config_.vbox_min_idx_[1]; y < tsdf_->map_config_.vbox_max_idx_[1];
         ++y) {
      for (int z = tsdf_->map_config_.vbox_min_idx_[2]; z < tsdf_->map_config_.vbox_max_idx_[2];
           ++z) {
        VoxelIndex idx(x, y, z);
        Position pos = occupancy_grid_->indexToPosition(idx);

        point.x = pos(0);
        point.y = pos(1);
        point.z = pos(2);

        // unknown or cleared voxel
        if (occupancy_grid_->getVoxel(idx).value == OccupancyType::OCCUPIED) {
          pointcloud_occupied.points.push_back(point);
        } else if (occupancy_grid_->getVoxel(idx).value == OccupancyType::FREE) {
          pointcloud_free.points.push_back(point);
        } else if (occupancy_grid_->getVoxel(idx).value == OccupancyType::UNKNOWN) {
          pointcloud_unknown.points.push_back(point);
        }
      }
    }
  }

  sensor_msgs::PointCloud2 pointcloud_msg;

  pointcloud_occupied.width = pointcloud_occupied.points.size();
  pointcloud_occupied.height = 1;
  pointcloud_occupied.is_dense = true;
  pointcloud_occupied.header.frame_id = config_.world_frame_;
  pcl::toROSMsg(pointcloud_occupied, pointcloud_msg);
  occupancy_grid_occupied_pub_.publish(pointcloud_msg);

  pointcloud_free.width = pointcloud_free.points.size();
  pointcloud_free.height = 1;
  pointcloud_free.is_dense = true;
  pointcloud_free.header.frame_id = config_.world_frame_;
  pcl::toROSMsg(pointcloud_free, pointcloud_msg);
  occupancy_grid_free_pub_.publish(pointcloud_msg);

  pointcloud_unknown.width = pointcloud_unknown.points.size();
  pointcloud_unknown.height = 1;
  pointcloud_unknown.is_dense = true;
  pointcloud_unknown.header.frame_id = config_.world_frame_;
  pcl::toROSMsg(pointcloud_unknown, pointcloud_msg);
  occupancy_grid_unknown_pub_.publish(pointcloud_msg);
}

void MapServer::publishOccupancyGridHighResolution() {
  // Publish occupancy grid with high resolution, e.g. 0.1m
  if (occupancy_grid_occupied_pub_.getNumSubscribers() == 0 &&
      occupancy_grid_free_pub_.getNumSubscribers() == 0 &&
      occupancy_grid_unknown_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO_THROTTLE(1.0, "[MapServer] Publishing occupancy grid...");

  PointCloudType pointcloud_occupied, pointcloud_free, pointcloud_unknown;
  PointType point;

  double map_resolution = getResolution();
  int scale = round(map_resolution / 0.1);

  for (int x = tsdf_->map_config_.vbox_min_idx_[0]; x < tsdf_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = tsdf_->map_config_.vbox_min_idx_[1]; y < tsdf_->map_config_.vbox_max_idx_[1];
         ++y) {
      for (int z = tsdf_->map_config_.vbox_min_idx_[2]; z < tsdf_->map_config_.vbox_max_idx_[2];
           ++z) {
        VoxelIndex idx(x, y, z);
        Position pos = occupancy_grid_->indexToPosition(idx);
        point.x = pos(0);
        point.y = pos(1);
        point.z = pos(2);

        // Expanded point to high resolution
        vector<PointType> points;
        PointType p1;
        p1.x = point.x - 0.05 * (scale - 1);
        p1.y = point.y - 0.05 * (scale - 1);
        p1.z = point.z - 0.05 * (scale - 1);
        for (int i = 0; i < scale; i++) {
          for (int j = 0; j < scale; j++) {
            for (int k = 0; k < scale; k++) {
              PointType p2 = p1;
              p2.x += 0.1 * i;
              p2.y += 0.1 * j;
              p2.z += 0.1 * k;
              points.push_back(p2);
            }
          }
        }

        // unknown or cleared voxel
        if (occupancy_grid_->getVoxel(idx).value == OccupancyType::OCCUPIED) {
          for (auto p : points)
            pointcloud_occupied.points.push_back(p);
        } else if (occupancy_grid_->getVoxel(idx).value == OccupancyType::FREE) {
          for (auto p : points)
            pointcloud_free.points.push_back(p);
        } else if (occupancy_grid_->getVoxel(idx).value == OccupancyType::UNKNOWN) {
          for (auto p : points)
            pointcloud_unknown.points.push_back(p);
        }
      }
    }
  }

  sensor_msgs::PointCloud2 pointcloud_msg;

  pointcloud_occupied.width = pointcloud_occupied.points.size();
  pointcloud_occupied.height = 1;
  pointcloud_occupied.is_dense = true;
  pointcloud_occupied.header.frame_id = config_.world_frame_;
  pcl::toROSMsg(pointcloud_occupied, pointcloud_msg);
  occupancy_grid_occupied_pub_.publish(pointcloud_msg);

  pointcloud_free.width = pointcloud_free.points.size();
  pointcloud_free.height = 1;
  pointcloud_free.is_dense = true;
  pointcloud_free.header.frame_id = config_.world_frame_;
  pcl::toROSMsg(pointcloud_free, pointcloud_msg);
  occupancy_grid_free_pub_.publish(pointcloud_msg);

  pointcloud_unknown.width = pointcloud_unknown.points.size();
  pointcloud_unknown.height = 1;
  pointcloud_unknown.is_dense = true;
  pointcloud_unknown.header.frame_id = config_.world_frame_;
  pcl::toROSMsg(pointcloud_unknown, pointcloud_msg);
  occupancy_grid_unknown_pub_.publish(pointcloud_msg);
}

void MapServer::publishOccupancyGridSlice() {
  if (occupancy_grid_slice_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing occupancy grid slice...");

  PointCloudType pointcloud;
  PointType point;

  for (int x = occupancy_grid_->map_config_.vbox_min_idx_[0];
       x < occupancy_grid_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = occupancy_grid_->map_config_.vbox_min_idx_[1];
         y < occupancy_grid_->map_config_.vbox_max_idx_[1]; ++y) {
      VoxelIndex idx(x, y, 0);
      Position pos = occupancy_grid_->indexToPosition(idx);
      pos.z() = config_.occupancy_grid_slice_height_;
      occupancy_grid_->positionToIndex(pos, idx);

      if (occupancy_grid_->getVoxel(idx).value == OccupancyType::UNKNOWN ||
          occupancy_grid_->getVoxel(idx).value == OccupancyType::FREE)
        continue;

      point.x = pos(0);
      point.y = pos(1);
      point.z = config_.occupancy_grid_slice_visualization_height_;
      pointcloud.points.push_back(point);
    }
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  occupancy_grid_slice_pub_.publish(pointcloud_msg);
}

void MapServer::publishDepthPointcloud(const PointCloudType &pointcloud,
                                       const ros::Time &img_stamp) {
  if (depth_pointcloud_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing depth pointcloud...");

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  pointcloud_msg.header.stamp = img_stamp;
  pointcloud_msg.header.frame_id = config_.world_frame_;
  depth_pointcloud_pub_.publish(pointcloud_msg);
}

void MapServer::publishInterpolatedPose(const Transformation &sensor_pose,
                                        const ros::Time &img_stamp) {
  nav_msgs::Odometry odo_msg;
  odo_msg.header.stamp = img_stamp;
  odo_msg.header.frame_id = config_.world_frame_;
  odo_msg.pose.pose.position.x = sensor_pose.getPosition().x();
  odo_msg.pose.pose.position.y = sensor_pose.getPosition().y();
  odo_msg.pose.pose.position.z = sensor_pose.getPosition().z();
  odo_msg.pose.pose.orientation.x = sensor_pose.getRotation().x();
  odo_msg.pose.pose.orientation.y = sensor_pose.getRotation().y();
  odo_msg.pose.pose.orientation.z = sensor_pose.getRotation().z();
  odo_msg.pose.pose.orientation.w = sensor_pose.getRotation().w();
  interpolated_pose_pub_.publish(odo_msg);
}

void MapServer::publishDebugVisualizationBBox(const Position &bbox_min, const Position &bbox_max) {
  if (debug_visualization_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing debug bbox...");

  visualization_msgs::Marker marker;
  marker.header.frame_id = config_.world_frame_;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = (bbox_min(0) + bbox_max(0)) / 2;
  marker.pose.position.y = (bbox_min(1) + bbox_max(1)) / 2;
  marker.pose.position.z = (bbox_min(2) + bbox_max(2)) / 2;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = bbox_max(0) - bbox_min(0);
  marker.scale.y = bbox_max(1) - bbox_min(1);
  marker.scale.z = bbox_max(2) - bbox_min(2);
  marker.color.a = 0.2;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  debug_visualization_pub_.publish(marker);
}

void MapServer::publishMapCoverage() {
  std_msgs::Float32 msg;
  int map_coverage_num = 0;
  for (int x = occupancy_grid_->map_config_.box_min_idx_.x();
       x < occupancy_grid_->map_config_.box_max_idx_.x(); x++) {
    for (int y = occupancy_grid_->map_config_.box_min_idx_.y();
         y < occupancy_grid_->map_config_.box_max_idx_.y(); y++) {
      for (int z = occupancy_grid_->map_config_.box_min_idx_.z();
           z < occupancy_grid_->map_config_.box_max_idx_.z(); z++) {
        VoxelIndex idx(x, y, z);
        if (occupancy_grid_->getVoxel(idx).value != OccupancyType::UNKNOWN) {
          map_coverage_num++;
        }
      }
    }
  }

  map_coverage_ = map_coverage_num * tsdf_->map_config_.resolution_ *
                  tsdf_->map_config_.resolution_ * tsdf_->map_config_.resolution_;
  msg.data = map_coverage_;
  map_coverage_pub_.publish(msg);

  // double total_volume =
  //     (occupancy_grid_->map_config_.box_max_ - occupancy_grid_->map_config_.box_min_).prod();
  // double accessibility = map_coverage_ / total_volume;
  // ROS_INFO_THROTTLE(5.0, "[MapServer] Map coverage: %.2f m^3, accessibility: %.2f%%",
  // map_coverage_,
  //                   accessibility * 100);
}

void MapServer::depthToPointcloud(const cv::Mat &depth_image, const Transformation &T_w_c,
                                  PointCloudType &pointcloud, const ros::Time &img_stamp) {
  Position point_c, point_w;
  const uint16_t *row_ptr;
  FloatingPoint depth;
  int cols = depth_image.cols;
  int rows = depth_image.rows;
  int points_num = 0;

  pointcloud.points.resize(cols * rows / (config_.skip_pixel_ * config_.skip_pixel_));
  pointcloud.sensor_origin_.head<3>() = T_w_c.getPosition().cast<float>();
  pointcloud.sensor_orientation_ = T_w_c.getRotationMatrix().cast<float>();

  for (int v = config_.depth_filter_margin_; v < rows - config_.depth_filter_margin_;
       v += config_.skip_pixel_) {
    //  Pointer to the first element (exclude the margin) in n-th row
    row_ptr = depth_image.ptr<uint16_t>(v) + config_.depth_filter_margin_;
    for (int u = config_.depth_filter_margin_; u < cols - config_.depth_filter_margin_;
         u += config_.skip_pixel_) {
      depth = (*row_ptr) * 0.001; // mm to m
      row_ptr = row_ptr + config_.skip_pixel_;
      if (depth == 0)
        continue;

      point_c.x() = (u - config_.cx_) * depth / config_.fx_;
      point_c.y() = (v - config_.cy_) * depth / config_.fy_;
      point_c.z() = depth;
      point_w = T_w_c * point_c;

      pointcloud.points[points_num].x = point_w.x();
      pointcloud.points[points_num].y = point_w.y();
      pointcloud.points[points_num].z = point_w.z();
      points_num++;

      // double z = -(v - config_.cy_) * depth / config_.fy_;
      // if (z < 0.1)
      //   continue;

      // // Point in camera frame
      // if (config_.mode_ == MapServer::Config::Mode::AIRSIM) {
      //   // x right, y forward, z up
      //   pointcloud.points[points_num].x = (u - config_.cx_) * depth / config_.fx_;
      //   pointcloud.points[points_num].y = depth;
      //   pointcloud.points[points_num].z = -(v - config_.cy_) * depth / config_.fy_;
      // } else {
      //   // x right, y down, z forward
      //   pointcloud.points[points_num].x = (u - config_.cx_) * depth / config_.fx_;
      //   pointcloud.points[points_num].y = (v - config_.cy_) * depth / config_.fy_;
      //   pointcloud.points[points_num].z = depth;
      // }
    }
  }

  pointcloud.points.resize(points_num);
  pointcloud.points.shrink_to_fit();

  publishDepthPointcloud(pointcloud, img_stamp);
}

void MapServer::depthToPointcloudDecimation(const cv::Mat &depth_image, const Transformation &T_w_c,
                                            PointCloudType &pointcloud,
                                            const ros::Time &img_stamp) {
  Position point_c, point_w;
  FloatingPoint depth;
  int cols = depth_image.cols;
  int rows = depth_image.rows;
  int points_num = 0;
  int decimation_magnitude_ = 10;

  pointcloud.points.resize(cols * rows / (decimation_magnitude_ * decimation_magnitude_));
  pointcloud.sensor_origin_.head<3>() = T_w_c.getPosition().cast<float>();
  pointcloud.sensor_orientation_ = T_w_c.getRotationMatrix().cast<float>();

  for (int v = 0; v < rows; v += decimation_magnitude_) {
    for (int u = 0; u < cols; u += decimation_magnitude_) {
      std::vector<uint16_t> kernel;
      for (int i = 0; i < decimation_magnitude_; i++) {
        for (int j = 0; j < decimation_magnitude_; j++) {
          if (v + i < rows && u + j < cols)
            kernel.push_back(depth_image.at<uint16_t>(v + i, u + j));
        }
      }

      std::sort(kernel.begin(), kernel.end());
      depth = kernel[kernel.size() / 2] * 0.001; // mm to m

      if (depth == 0)
        continue;

      point_c.x() = (u - config_.cx_) * depth / config_.fx_;
      point_c.y() = (v - config_.cy_) * depth / config_.fy_;
      point_c.z() = depth;
      point_w = T_w_c * point_c;

      // x right, y down, z forward
      pointcloud.points[points_num].x = point_w.x();
      pointcloud.points[points_num].y = point_w.y();
      pointcloud.points[points_num].z = point_w.z();
      points_num++;
    }
  }

  pointcloud.points.resize(points_num);
  pointcloud.points.shrink_to_fit();

  publishDepthPointcloud(pointcloud, img_stamp);
}

bool MapServer::getNextImageFromQueue(std::queue<sensor_msgs::ImageConstPtr> &queue,
                                      cv::Mat &depth_image, Transformation &sensor_pose,
                                      ros::Time &img_stamp) {
  if (queue.empty()) {
    return false;
  }

  if (config_.verbose_)
    ROS_INFO("[MapServer] Getting next image from queue...");

  sensor_msgs::ImageConstPtr image_msg;
  const int kMaxQueueSize = 100;

  image_msg = queue.front();
  if (transformer_->lookupTransform(image_msg->header.stamp, sensor_pose)) {
    if (config_.verbose_)
      ROS_INFO("[MapServer] Found sensor pose for image timestamp: %f",
               image_msg->header.stamp.toSec());

    // TODO: maybe no need for cv::Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    if (image_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, config_.depth_scaling_factor_);
    cv_ptr->image.copyTo(depth_image);
    img_stamp = image_msg->header.stamp;

    queue.pop();
    return true;
  } else {
    if (config_.verbose_)
      ROS_ERROR("[MapServer] Failed to lookup transform for image");
    if (queue.size() >= kMaxQueueSize) {
      if (config_.verbose_)
        ROS_ERROR("[MapServer] Queue size is too large, dropping oldest image");
      while (queue.size() >= kMaxQueueSize) {
        if (config_.verbose_)
          ROS_ERROR("[MapServer] Pop image with timestamp: %f",
                    queue.front()->header.stamp.toSec());
        queue.pop();
      }
    }
  }

  return false;
}

bool MapServer::getNextPointcloudFromQueue(std::queue<sensor_msgs::PointCloud2ConstPtr> &queue,
                                           PointCloudType &pointcloud, Transformation &sensor_pose,
                                           ros::Time &pcl_stamp) {
  if (queue.empty()) {
    return false;
  }

  if (config_.verbose_)
    ROS_INFO("[MapServer] Getting next image from queue...");

  sensor_msgs::PointCloud2ConstPtr pointcloud_msg;
  const int kMaxQueueSize = 10;

  pointcloud_msg = queue.front();
  if (transformer_->lookupTransform(pointcloud_msg->header.stamp, sensor_pose)) {
    if (config_.verbose_)
      ROS_INFO("[MapServer] Found sensor pose for pointcloud timestamp: %f",
               pointcloud_msg->header.stamp.toSec());

    pcl::fromROSMsg(*pointcloud_msg, pointcloud);
    pcl_stamp = pointcloud_msg->header.stamp;

    pointcloud.sensor_origin_.head<3>() = sensor_pose.getPosition().cast<float>();
    pointcloud.sensor_orientation_ = sensor_pose.getRotationMatrix().cast<float>();

    queue.pop();
    return true;
  } else {
    if (config_.verbose_)
      ROS_ERROR("[MapServer] Failed to lookup transform for pointcloud");
    if (queue.size() >= kMaxQueueSize) {
      if (config_.verbose_)
        ROS_ERROR("[MapServer] Queue size is too large, dropping oldest pointcloud");
      while (queue.size() >= kMaxQueueSize) {
        if (config_.verbose_)
          ROS_ERROR("[MapServer] Pop pointcloud with timestamp: %f",
                    queue.front()->header.stamp.toSec());
        queue.pop();
      }
    }
  }

  return false;
}

void MapServer::pointcloudTransform(const PointCloudType &pointcloud_sensor,
                                    const Transformation &T_w_c, PointCloudType &pointcloud_world) {
  pointcloud_world.clear();
  pointcloud_world.points.resize(pointcloud_sensor.size());

  for (size_t i = 0; i < pointcloud_sensor.size(); i++) {
    Position point_sensor, point_world;
    point_sensor.x() = pointcloud_sensor.points[i].x;
    point_sensor.y() = pointcloud_sensor.points[i].y;
    point_sensor.z() = pointcloud_sensor.points[i].z;
    point_world = T_w_c * point_sensor;
    pointcloud_world.points[i] = PointType(point_world.x(), point_world.y(), point_world.z());
  }

  // pointcloud_world.sensor_origin_ = T_w_c.getPosition().cast<float>();
  // pointcloud_world.sensor_orientation_ = T_w_c.getRotationMatrix().cast<float>();

  pointcloud_world.width = pointcloud_world.points.size();
  pointcloud_world.height = 1;
  pointcloud_world.is_dense = true;
  pointcloud_world.header.frame_id = config_.world_frame_;
}

// Old API
void MapServer::getRegion(Position &ori, Eigen::Vector3d &size) {
  ori = tsdf_->map_config_.map_min_;
  size = tsdf_->map_config_.map_size_;
}

Position MapServer::getOrigin() { return tsdf_->map_config_.map_min_; }

double MapServer::getResolution() { return tsdf_->map_config_.resolution_; };

int MapServer::getVoxelNum() { return tsdf_->map_config_.map_size_idx_.prod(); };

void MapServer::getBox(Position &bmin, Position &bmax) {
  bmin = tsdf_->map_config_.box_min_;
  bmax = tsdf_->map_config_.box_max_;
}

void MapServer::getBoxIndex(VoxelIndex &bmin_idx, VoxelIndex &bmax_idx) {
  bmin_idx = tsdf_->map_config_.box_min_idx_;
  bmax_idx = tsdf_->map_config_.box_max_idx_;
}

bool MapServer::isInBox(const Position &pos) { return tsdf_->isInBox(pos); };
bool MapServer::isInBox(const VoxelIndex &idx) { return tsdf_->isInBox(idx); };

OccupancyType MapServer::getOccupancy(const Position &pos) {
  return occupancy_grid_->getVoxel(pos).value;
}

OccupancyType MapServer::getOccupancy(const VoxelIndex &idx) {
  return occupancy_grid_->getVoxel(idx).value;
}

double MapServer::getMapCoverage() { return map_coverage_; }

void MapServer::saveMap(const string &filename) {
  std::cout << "[MapServer] Saving map to " << filename << std::endl;
  occupancy_grid_->saveMap(filename);
  std::cout << "[MapServer] Done" << std::endl;
}

void MapServer::saveMapToPCD(const string &filename) {
  std::cout << "[MapServer] Saving map to PCD " << filename << std::endl;
  occupancy_grid_->saveMapToPCD(filename);
  std::cout << "[MapServer] Done" << std::endl;
}

void MapServer::loadMap(const string &filename) {
  std::cout << "[MapServer] Loading map from " << filename << std::endl;
  if (filename.find(".pcd") != std::string::npos)
    occupancy_grid_->loadMapFromPCD(filename);
  else if (filename.find(".ply") != std::string::npos)
    occupancy_grid_->loadMapFromPLY(filename);
  else
    occupancy_grid_->loadMap(filename);
  std::cout << "[MapServer] Done" << std::endl;
}

void MapServer::loadMap(const string &filename_occupied, const string &filename_free) {
  std::cout << "[MapServer] Loading occupied pcl from " << filename_occupied << std::endl;
  occupancy_grid_->loadMapFromPCD(filename_occupied, OccupancyType::OCCUPIED);

  std::cout << "[MapServer] Loading free pcl from " << filename_free << std::endl;
  occupancy_grid_->loadMapFromPCD(filename_free, OccupancyType::FREE);

  std::cout << "[MapServer] Done" << std::endl;
}

void MapServer::loadMapFromPCL(const PointCloudType::Ptr cloud) {
  ROS_INFO("[MapServer] Loading map from PCL");

  occupancy_grid_->loadMapFromPCL(cloud, OccupancyType::OCCUPIED, Eigen::Vector3i(1, 1, 1), true);
  ROS_INFO("[MapServer] Occupancy grid loaded");

  PointCloudType::Ptr cloud_occ(new PointCloudType);
  PointCloudType::Ptr cloud_free(new PointCloudType);
  occupancy_grid_->getOccupancyPointcloud(cloud_occ, OccupancyType::OCCUPIED);
  occupancy_grid_->getOccupancyPointcloud(cloud_free, OccupancyType::FREE);
  esdf_->loadMapFromPCL(cloud_occ, cloud_free);
  ROS_INFO("[MapServer] ESDF loaded");

  // occupancy_grid_->saveMap("/home/eason/occu.txt");
  // esdf_->saveMap("/home/eason/esdf.txt");

  // occupancy_grid_->loadMap("/home/eason/occu.txt");
  // esdf_->loadMap("/home/eason/esdf.txt");
}

void MapServer::scaleColor(const double value, const double max_value, const double min_value,
                           double &color_value) {
  // Scale the value to the range [0, 1]
  double scaled_value = (value - min_value) / (max_value - min_value);
  // Apply the color map to convert the value to a grayscale color
  double color_map_scale = 1.0;
  color_value = color_map_scale * scaled_value;
}
} // namespace voxel_mapping