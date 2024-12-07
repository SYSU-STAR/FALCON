#include "perception_utils/perception_utils.h"

namespace fast_planner {
PerceptionUtils::PerceptionUtils(ros::NodeHandle &nh) {
  double horizontal_fov, vertical_fov;
  nh.param("/uav_model/sensing_parameters/fov/horizontal", horizontal_fov, 0.0);
  nh.param("/uav_model/sensing_parameters/fov/vertical", vertical_fov, 0.0);
  left_angle_ = horizontal_fov / 2.0;
  right_angle_ = horizontal_fov / 2.0;
  top_angle_ = vertical_fov / 2.0;
  bottom_angle_ = vertical_fov / 2.0;

  std::string fov_type_str;
  nh.param("/perception_utils/fov_type", fov_type_str, std::string("pinhole"));
  nh.param("/perception_utils/max_dist", max_dist_, -1.0);
  nh.param("/perception_utils/vis_dist", vis_dist_, -1.0);

  if (fov_type_str == "pinhole") {
    fov_type_ = PINHOLE;
  } else if (fov_type_str == "omni_lidar") {
    fov_type_ = OMNI_LIDAR;
  } else {
    ROS_ERROR("[PerceptionUtils] Unknown FOV type: %s", fov_type_str.c_str());
  }

  if (fov_type_ == PINHOLE) {
    n_top_ << 0.0, sin(M_PI_2 - top_angle_), cos(M_PI_2 - top_angle_);
    n_bottom_ << 0.0, -sin(M_PI_2 - bottom_angle_), cos(M_PI_2 - bottom_angle_);
    n_left_ << sin(M_PI_2 - left_angle_), 0.0, cos(M_PI_2 - left_angle_);
    n_right_ << -sin(M_PI_2 - right_angle_), 0.0, cos(M_PI_2 - right_angle_);

    T_cb_ << 0, -1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    T_bc_ = T_cb_.inverse();

    // FOV vertices in body frame, for FOV visualization
    double hor = vis_dist_ * tan(left_angle_);
    double vert = vis_dist_ * tan(top_angle_);
    Vector3d origin(0, 0, 0);
    Vector3d left_up(vis_dist_, hor, vert);
    Vector3d left_down(vis_dist_, hor, -vert);
    Vector3d right_up(vis_dist_, -hor, vert);
    Vector3d right_down(vis_dist_, -hor, -vert);

    cam_vertices1_.push_back(origin);
    cam_vertices2_.push_back(left_up);
    cam_vertices1_.push_back(origin);
    cam_vertices2_.push_back(left_down);
    cam_vertices1_.push_back(origin);
    cam_vertices2_.push_back(right_up);
    cam_vertices1_.push_back(origin);
    cam_vertices2_.push_back(right_down);

    cam_vertices1_.push_back(left_up);
    cam_vertices2_.push_back(right_up);
    cam_vertices1_.push_back(right_up);
    cam_vertices2_.push_back(right_down);
    cam_vertices1_.push_back(right_down);
    cam_vertices2_.push_back(left_down);
    cam_vertices1_.push_back(left_down);
    cam_vertices2_.push_back(left_up);

    // Discretize cam FOV frustum to voxels with resolution in body frame frame
    cam_fov_voxels_.clear();
    double x = max_dist_;
    double y_max = x * tan(left_angle_);
    double z_max = x * tan(top_angle_);
    for (double y = -y_max; y <= y_max + 1e-4; y += y_max / 3.0) {
      for (double z = -z_max; z <= z_max + 1e-4; z += z_max / 3.0) {
        cam_fov_voxels_.push_back(Vector3d(x, y, z));
      }
    }
  }
}

void PerceptionUtils::setPose(const Vector3d &pos, const double &yaw) {
  pos_ = pos;
  yaw_ = yaw;

  // Transform the normals of camera FOV
  Eigen::Matrix3d R_wb;
  R_wb << cos(yaw_), -sin(yaw_), 0.0, sin(yaw_), cos(yaw_), 0.0, 0.0, 0.0, 1.0;
  Vector3d pc = pos_;

  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity();
  T_wb.block<3, 3>(0, 0) = R_wb;
  T_wb.block<3, 1>(0, 3) = pc;
  Eigen::Matrix4d T_wc = T_wb * T_bc_;
  Eigen::Matrix3d R_wc = T_wc.block<3, 3>(0, 0);
  // Vector3d t_wc = T_wc.block<3, 1>(0, 3);
  normals_ = {n_top_, n_bottom_, n_left_, n_right_};
  for (auto &n : normals_)
    n = R_wc * n;
}

void PerceptionUtils::getFOV(vector<Vector3d> &list1, vector<Vector3d> &list2) {
  if (fov_type_ == OMNI_LIDAR) {
    ROS_ERROR("[PerceptionUtils] Omni-lidar FOV visualization not implemented yet.");
    return;
    return;
  }
  list1.clear();
  list2.clear();

  // Get info for visualizing FOV at (pos, yaw)
  Eigen::Matrix3d Rwb;
  Rwb << cos(yaw_), -sin(yaw_), 0, sin(yaw_), cos(yaw_), 0, 0, 0, 1;
  for (size_t i = 0; i < cam_vertices1_.size(); ++i) {
    auto p1 = Rwb * cam_vertices1_[i] + pos_;
    auto p2 = Rwb * cam_vertices2_[i] + pos_;
    list1.push_back(p1);
    list2.push_back(p2);
  }
}

bool PerceptionUtils::insideFOV(const Vector3d &point) {
  if (fov_type_ == PINHOLE)
    return insidePinholeFOV(point);
  else if (fov_type_ == OMNI_LIDAR)
    return insideOmniLidarFOV(point);
  else {
    ROS_ERROR("[PerceptionUtils] Unknown FOV type: %d", fov_type_);
  }

  return false;
}

bool PerceptionUtils::insidePinholeFOV(const Vector3d &point) {
  Eigen::Vector3d dir = point - pos_;
  if (dir.norm() > max_dist_)
    return false;

  dir.normalize();
  for (auto n : normals_) {
    if (dir.dot(n) < 0.0)
      return false;
  }
  return true;
}

bool PerceptionUtils::insideOmniLidarFOV(const Vector3d &point) {
  Eigen::Vector3d dir = point - pos_;
  if (dir.norm() > max_dist_)
    return false;

  dir.normalize();
  double angle = atan2(dir(2), sqrt(dir(0) * dir(0) + dir(1) * dir(1)));
  if (angle > -bottom_angle_ && angle < top_angle_)
    return true;
  return false;
}
} // namespace fast_planner