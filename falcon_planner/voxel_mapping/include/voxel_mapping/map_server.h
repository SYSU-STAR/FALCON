#ifndef TSDF_SERVER_H
#define TSDF_SERVER_H

#include <atomic>
#include <mutex>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "tic_toc.h"
#include "transformer/transformer.h"
#include "voxel_mapping/esdf.h"
#include "voxel_mapping/occupancy_grid.h"
#include "voxel_mapping/tsdf.h"

using std::shared_ptr;
using std::string;

namespace voxel_mapping {
class MapServer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<MapServer> Ptr;
  typedef shared_ptr<const MapServer> ConstPtr;

  struct Config {
    enum Mode { AIRSIM, UAV_SIMULATOR, REAL };

    Mode mode_;
    double fx_, fy_, cx_, cy_;
    int image_width_, image_height_;
    int depth_filter_margin_;
    double depth_scaling_factor_, depth_scaling_factor_inv_;
    bool depth_decimation_filter_;
    int skip_pixel_;
    int concurrent_depth_input_max_, concurrent_pointcloud_input_max_;

    bool publish_tsdf_, publish_esdf_, publish_occupancy_grid_;
    bool publish_tsdf_slice_, publish_esdf_slice_, publish_occupancy_grid_slice_;
    double publish_tsdf_period_, publish_esdf_period_, publish_occupancy_grid_period_;
    double tsdf_slice_height_, esdf_slice_height_, occupancy_grid_slice_height_;
    double tsdf_slice_visualization_height_, esdf_slice_visualization_height_,
        occupancy_grid_slice_visualization_height_;

    string world_frame_, sensor_frame_;

    bool verbose_, verbose_time_;

    // Print all parameters
    void print() {
      std::cout << "-------------------" << std::endl;
      std::cout << "Mode: " << mode_ << std::endl;
      std::cout << "fx: " << fx_ << std::endl;
      std::cout << "fy: " << fy_ << std::endl;
      std::cout << "cx: " << cx_ << std::endl;
      std::cout << "cy: " << cy_ << std::endl;
      std::cout << "image_width: " << image_width_ << std::endl;
      std::cout << "image_height: " << image_height_ << std::endl;
      std::cout << "depth_filter_margin: " << depth_filter_margin_ << std::endl;
      std::cout << "depth_scaling_factor: " << depth_scaling_factor_ << std::endl;
      std::cout << "depth_scaling_factor_inv: " << depth_scaling_factor_inv_ << std::endl;
      std::cout << "depth_decimation_filter: " << depth_decimation_filter_ << std::endl;
      std::cout << "skip_pixel: " << skip_pixel_ << std::endl;
      std::cout << "concurrent_depth_input_max: " << concurrent_depth_input_max_ << std::endl;
      std::cout << "concurrent_pointcloud_input_max: " << concurrent_pointcloud_input_max_
                << std::endl;
      std::cout << "publish_tsdf: " << publish_tsdf_ << std::endl;
      std::cout << "publish_esdf: " << publish_esdf_ << std::endl;
      std::cout << "publish_occupancy_grid: " << publish_occupancy_grid_ << std::endl;
      std::cout << "publish_tsdf_slice: " << publish_tsdf_slice_ << std::endl;
      std::cout << "publish_esdf_slice: " << publish_esdf_slice_ << std::endl;
      std::cout << "publish_occupancy_grid_slice: " << publish_occupancy_grid_slice_ << std::endl;
      std::cout << "publish_tsdf_period: " << publish_tsdf_period_ << std::endl;
      std::cout << "publish_esdf_period: " << publish_esdf_period_ << std::endl;
      std::cout << "publish_occupancy_grid_period: " << publish_occupancy_grid_period_ << std::endl;
      std::cout << "tsdf_slice_height: " << tsdf_slice_height_ << std::endl;
      std::cout << "esdf_slice_height: " << esdf_slice_height_ << std::endl;
      std::cout << "occupancy_grid_slice_height: " << occupancy_grid_slice_height_ << std::endl;
      std::cout << "tsdf_slice_visualization_height: " << tsdf_slice_visualization_height_
                << std::endl;
      std::cout << "esdf_slice_visualization_height: " << esdf_slice_visualization_height_
                << std::endl;
      std::cout << "occupancy_grid_slice_visualization_height: "
                << occupancy_grid_slice_visualization_height_ << std::endl;
      std::cout << "world_frame: " << world_frame_ << std::endl;
      std::cout << "sensor_frame: " << sensor_frame_ << std::endl;
      std::cout << "verbose: " << verbose_ << std::endl;
      std::cout << "verbose_time: " << verbose_time_ << std::endl;
      std::cout << "-------------------" << std::endl;
    }
  };

  MapServer(ros::NodeHandle &nh);

  TSDF::Ptr getTSDF() { return tsdf_; }
  ESDF::Ptr getESDF() { return esdf_; }
  OccupancyGrid::Ptr getOccupancyGrid() { return occupancy_grid_; }
  Transformer::Ptr getTransformer() { return transformer_; }
  std::mutex &getMapMutex() { return map_mutex_; }

  void depthCallback(const sensor_msgs::ImageConstPtr &image_msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg);
  void globalMapCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg);

  // Debug
  bool saveMapPCDCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  void testESDFCallback(const geometry_msgs::PoseStamped &msg);

  void publishMapTimerCallback(const ros::TimerEvent &event);
  void publishTSDF();
  void publishTSDFSlice();
  void publishESDF();
  void publishESDFSlice();
  void publishOccupancyGrid();
  void publishOccupancyGridHighResolution();
  void publishOccupancyGridSlice();
  void publishDepthPointcloud(const PointCloudType &pointcloud, const ros::Time &img_stamp);
  void publishInterpolatedPose(const Transformation &sensor_pose, const ros::Time &img_stamp);
  void publishDebugVisualizationBBox(const Position &bbox_min, const Position &bbox_max);
  void publishMapCoverage();

  // Transform a depth image to a point cloud under sensor frame
  // sensor pose is stored in pointcloud sensor_origin_ and sensor_orientation_
  void depthToPointcloud(const cv::Mat &depth_image, const Transformation &T_w_c,
                         PointCloudType &pointcloud, const ros::Time &img_stamp);
  void depthToPointcloudDecimation(const cv::Mat &depth_image, const Transformation &T_w_c,
                                   PointCloudType &pointcloud, const ros::Time &img_stamp);
  bool getNextImageFromQueue(std::queue<sensor_msgs::ImageConstPtr> &queue, cv::Mat &depth_image,
                             Transformation &sensor_pose, ros::Time &img_stamp);
  bool getNextPointcloudFromQueue(std::queue<sensor_msgs::PointCloud2ConstPtr> &queue,
                                  PointCloudType &pointcloud, Transformation &sensor_pose,
                                  ros::Time &pcl_stamp);

  void pointcloudTransform(const PointCloudType &pointcloud_sensor, const Transformation &T_w_c,
                           PointCloudType &pointcloud_world);

  void getRegion(Position &ori, Eigen::Vector3d &size);
  Position getOrigin();
  double getResolution();
  int getVoxelNum();
  void getBox(Position &bmin, Position &bmax);
  void getBoxIndex(VoxelIndex &bmin_idx,VoxelIndex &bmax_idx);
  bool isInBox(const Position &pos);
  bool isInBox(const VoxelIndex &idx);
  OccupancyType getOccupancy(const Position &pos);
  OccupancyType getOccupancy(const VoxelIndex &idx);
  double getMapCoverage();

  void saveMap(const string &filename);
  void saveMapToPCD(const string &filename);
  void loadMap(const string &filename);
  void loadMap(const string &filename_occupied, const string &filename_free);
  void loadMapFromPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void scaleColor(const double value, const double max_value, const double min_value,
                  double &color_value);

private:
  Config config_;

  ros::Subscriber depth_sub_, pointcloud_sub_, global_map_sub_;

  ros::Publisher tsdf_pub_, tsdf_slice_pub_;
  ros::Publisher esdf_pub_, esdf_slice_pub_;
  ros::Publisher occupancy_grid_occupied_pub_, occupancy_grid_free_pub_,
      occupancy_grid_unknown_pub_, occupancy_grid_slice_pub_;
  ros::Publisher interpolated_pose_pub_;
  ros::Publisher depth_pointcloud_pub_;
  ros::Publisher map_coverage_pub_;
  ros::Publisher debug_visualization_pub_;

  ros::Timer publish_map_timer_;

  ros::ServiceServer save_map_srv_;

  TSDF::Ptr tsdf_;
  ESDF::Ptr esdf_;
  OccupancyGrid::Ptr occupancy_grid_;
  Transformer::Ptr transformer_;

  std::mutex map_mutex_;
  std::atomic<int> map_update_counter_;

  std::queue<sensor_msgs::ImageConstPtr> image_queue_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> pointcloud_queue_;

  Position pos_min_, pos_max_;
  bool load_map_from_pcd_;
  int counter_;
  double map_coverage_;
};
} // namespace voxel_mapping

#endif // TSDF_SERVER_H