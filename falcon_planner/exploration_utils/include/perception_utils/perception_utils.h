#ifndef _PERCEPTION_UTILS_H_
#define _PERCEPTION_UTILS_H_

#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class PerceptionUtils {
public:
  typedef shared_ptr<PerceptionUtils> Ptr;
  typedef shared_ptr<const PerceptionUtils> ConstPtr;

  enum FOVTpye { PINHOLE, OMNI_LIDAR };

  PerceptionUtils(ros::NodeHandle &nh);
  ~PerceptionUtils() {}

  // Set position and yaw
  void setPose(const Vector3d &pos, const double &yaw);

  // Get info of current pose
  void getFOV(vector<Vector3d> &list1, vector<Vector3d> &list2);
  bool insideFOV(const Vector3d &point);
  bool insidePinholeFOV(const Vector3d &point);
  bool insideOmniLidarFOV(const Vector3d &point);

  vector<Vector3d> cam_fov_voxels_;

private:
  // Data
  // Current camera pos and yaw
  Vector3d pos_;
  double yaw_;
  // Camera plane's normals in world frame
  vector<Vector3d> normals_;

  /* Params */
  // FOV of RealSense D435: 86 deg x 57 deg (HD 16:9), max depth distance: 5m
  // FOV of Mid-360: 360 deg x 59 deg, vertical angle: -7 deg ~ 52 deg, max depth distance: 5m
  // vis_dist_: visualization FOV distance, typically 0.5~1.0 m
  FOVTpye fov_type_;
  double left_angle_, right_angle_, top_angle_, bottom_angle_, max_dist_, vis_dist_;

  // Pin-hole camera FOV
  // Normal vectors of camera FOV planes in camera frame
  Vector3d n_top_, n_bottom_, n_left_, n_right_;
  // Transform between camera and body
  Eigen::Matrix4d T_cb_, T_bc_;
  // FOV vertices in body frame
  vector<Vector3d> cam_vertices1_, cam_vertices2_;
};
} // namespace fast_planner
#endif