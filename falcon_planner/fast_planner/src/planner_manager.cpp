#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

#include "fast_planner/planner_manager.h"

namespace fast_planner {
void FastPlannerManager::init(ros::NodeHandle &nh) {
  // Load parameters
  nh.param("/uav_model/dynamics_parameters/max_linear_velocity", pp_.max_vel_, -1.0);
  nh.param("/uav_model/dynamics_parameters/max_linear_acceleration", pp_.max_acc_, -1.0);

  nh.param("/fast_planner/control_points_distance", pp_.ctrl_pt_dist_, -1.0);

  // Check parameters
  CHECK(pp_.max_vel_ > 0 && pp_.max_acc_ > 0 && pp_.ctrl_pt_dist_ > 0)
      << "Invalid parameters: max_vel = " << pp_.max_vel_ << ", max_acc = " << pp_.max_acc_
      << ", ctrl_pt_dist = " << pp_.ctrl_pt_dist_;

  local_data_.traj_id_ = 0;
  map_server_.reset(new voxel_mapping::MapServer(nh));

  double resolution = map_server_->getResolution();
  Eigen::Vector3d origin, size;
  map_server_->getRegion(origin, size);
  caster_.reset(new RayCaster);
  caster_->setParams(resolution, origin);

  path_finder_.reset(new Astar);
  path_finder_->init(nh, map_server_);

  // 2 Bspline optimizers for position and yaw
  bspline_optimizers_.resize(2);
  for (size_t i = 0; i < bspline_optimizers_.size(); ++i) {
    bspline_optimizers_[i].reset(new BsplineOptimizer);
    bspline_optimizers_[i]->setParam(nh);
    bspline_optimizers_[i]->setMap(map_server_);
    if (i == 0)
      // Position optimizer physical limits
      bspline_optimizers_[i]->setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  }
}

void FastPlannerManager::planExplorationPositionTraj(const vector<Eigen::Vector3d> &tour,
                                                     const Eigen::Vector3d &cur_vel,
                                                     const Eigen::Vector3d &cur_acc,
                                                     const double &time_lb, const bool verbose) {
  if (tour.empty())
    ROS_ERROR("[FastPlannerManager] Empty path for trajecotry planning");

  ros::Time t1 = ros::Time::now();

  Eigen::Vector3d cur_vel_bound = cur_vel;
  for (int i = 0; i < 3; ++i) {
    cur_vel_bound(i) = std::min(pp_.max_vel_, std::max(-pp_.max_vel_, cur_vel(i)));
  }

  Eigen::Vector3d cur_acc_bound = cur_acc;
  for (int i = 0; i < 3; ++i) {
    cur_acc_bound(i) = std::min(pp_.max_acc_, std::max(-pp_.max_acc_, cur_acc(i)));
  }

  if (verbose) {
    // Print current states
    std::cout << "cur_vel: " << cur_vel.transpose() << std::endl;
    std::cout << "cur_vel_bound: " << cur_vel_bound.transpose() << std::endl;
    std::cout << "cur_acc: " << cur_acc.transpose() << std::endl;
    std::cout << "cur_acc_bound: " << cur_acc_bound.transpose() << std::endl;

    // Print tour
    std::cout << "tour size: " << tour.size() << std::endl;
    for (int i = 0; i < tour.size(); ++i) {
      std::cout << "tour[" << i << "]: " << tour[i].transpose() << std::endl;
    }
  }

  // Generate traj through waypoints-based method
  const int pt_num = tour.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i)
    pos.row(i) = tour[i];

  Eigen::VectorXd times(pt_num - 1); // time for each segment on tour
  for (int i = 0; i < pt_num - 1; ++i) {
    times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);

    if (i == 0) {
      // consider cur_vel for first segment
      Vector3d dir;
      dir = (pos.row(1) - pos.row(0)).normalized();
      double vc = cur_vel.dot(dir);
      double t = pow(pp_.max_vel_ - fabs(vc), 2) / (2 * pp_.max_vel_ * pp_.max_acc_);

      if (vc < 0) {
        t += 2 * fabs(vc) / pp_.max_acc_;
      }
      times(i) += t;
    }

    if (i == pt_num - 2) {
      // consider vel change for last segment to zero
      double vc = pp_.max_vel_;
      double t = pow(0 - fabs(vc), 2) / (2 * pp_.max_vel_ * pp_.max_acc_);

      if (vc < 0) {
        t += 2 * fabs(vc) / pp_.max_acc_;
      }
      times(i) += t;
    }
  }

  // Print times
  if (verbose) {
    for (int i = 0; i < pt_num - 1; ++i) {
      std::cout << "times[" << i << "]: " << times(i) << std::endl;
    }
  }

  double times_sum = times.sum();
  if (times_sum < time_lb) {
    double scale = time_lb / times_sum;
    times *= scale;
  }

  bool need_optimize = true;

  PolynomialTraj::waypointsTraj(pos, cur_vel_bound, Eigen::Vector3d::Zero(), cur_acc_bound,
                                Eigen::Vector3d::Zero(), times, local_data_.init_traj_poly_);
  PolynomialTraj &init_traj = local_data_.init_traj_poly_;

  if (tour.size() == 3 && (tour[1] - tour[0]).norm() + (tour[2] - tour[1]).norm() < 1.0) {
    ROS_WARN("[FastPlannerManager] Short path, no need to optimize.");
    need_optimize = false;

    // Hack for short tour glitch
    double tour_len = (tour[1] - tour[0]).norm() + (tour[2] - tour[1]).norm();
    if (init_traj.getLength() > 5.0 * tour_len) {
      ROS_WARN("[FastPlannerManager] Short tour glitch detected, recalculating trajectory.");
      // Static state
      for (int i = 0; i < pt_num - 1; ++i) {
        times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);
      }
      double times_sum = times.sum();
      if (times_sum < time_lb) {
        double scale = time_lb / times_sum;
        times *= scale;
      }
      PolynomialTraj::waypointsTraj(pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times,
                                    local_data_.init_traj_poly_);
    }
  }

  // B-spline-based optimization
  vector<Vector3d> points, boundary_deri;
  double duration = init_traj.getTotalTime();
  int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist_;
  seg_num = max(8, seg_num);
  double dt = duration / double(seg_num);

  // std::cout << "duration: " << duration << ", seg_num: " << seg_num << ", dt: " << dt <<
  // std::endl;

  for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
    points.push_back(init_traj.evaluate(ts, 0));
  // Evaluate velocity at start and end
  boundary_deri.push_back(init_traj.evaluate(0.0, 1));
  boundary_deri.push_back(init_traj.evaluate(duration, 1));
  // Evaluate acceleration at start and end
  boundary_deri.push_back(init_traj.evaluate(0.0, 2));
  boundary_deri.push_back(init_traj.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  int bspline_degree = bspline_optimizers_[0]->getBsplineDegree();
  NonUniformBspline::parameterizeToBspline(dt, points, boundary_deri, bspline_degree, ctrl_pts);
  local_data_.init_traj_bspline_ = NonUniformBspline(ctrl_pts, bspline_degree, dt);

  int cost_func = BsplineOptimizer::EXPLORATION_POSITION_PHASE;

  vector<Vector3d> start, end;
  local_data_.init_traj_bspline_.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  if (time_lb > 0)
    bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  if (need_optimize) {
    // bspline optimizer generate extreme slow trajectory for short tours
    bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func);
  }

  local_data_.position_traj_.setUniformBspline(ctrl_pts, bspline_degree, dt);
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

  local_data_.duration_ = local_data_.position_traj_.getTimeSum();
  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.end_pos_ = local_data_.position_traj_.evaluateDeBoorT(local_data_.duration_);

  local_data_.traj_id_ += 1;

  local_data_.tour_ = tour;
  local_data_.cur_vel_ = cur_vel;
  local_data_.cur_acc_ = cur_acc;
  local_data_.time_lb_ = time_lb;

  if (verbose) {
    // log trajectory data for debugging
    LOG(INFO) << "[FastPlannerManager] Position trajectory planned, id: " << local_data_.traj_id_
              << ", duration: " << local_data_.duration_
              << ", start: " << local_data_.start_pos_.transpose()
              << ", end: " << local_data_.end_pos_.transpose() << ", optimized: " << need_optimize;
    // log sampled velocity
    std::vector<double> vel_x, vel_y, vel_z;
    for (double t = 0.0; t <= local_data_.duration_; t += 0.1) {
      Eigen::Vector3d vel = local_data_.velocity_traj_.evaluateDeBoorT(t);
      vel_x.push_back(vel(0));
      vel_y.push_back(vel(1));
      vel_z.push_back(vel(2));
    }
    std::string vel_x_str =
        std::accumulate(vel_x.begin(), vel_x.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    std::string vel_y_str =
        std::accumulate(vel_y.begin(), vel_y.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    std::string vel_z_str =
        std::accumulate(vel_z.begin(), vel_z.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    LOG(INFO) << "[FastPlannerManager] Sampled velocity x: " << vel_x_str;
    LOG(INFO) << "[FastPlannerManager] Sampled velocity y: " << vel_y_str;
    LOG(INFO) << "[FastPlannerManager] Sampled velocity z: " << vel_z_str;

    // log sampled acceleration
    std::vector<double> acc_x, acc_y, acc_z;
    for (double t = 0.0; t <= local_data_.duration_; t += 0.1) {
      Eigen::Vector3d acc = local_data_.acceleration_traj_.evaluateDeBoorT(t);
      acc_x.push_back(acc(0));
      acc_y.push_back(acc(1));
      acc_z.push_back(acc(2));
    }
    std::string acc_x_str =
        std::accumulate(acc_x.begin(), acc_x.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    std::string acc_y_str =
        std::accumulate(acc_y.begin(), acc_y.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    std::string acc_z_str =
        std::accumulate(acc_z.begin(), acc_z.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    LOG(INFO) << "[FastPlannerManager] Sampled acceleration x: " << acc_x_str;
    LOG(INFO) << "[FastPlannerManager] Sampled acceleration y: " << acc_y_str;
    LOG(INFO) << "[FastPlannerManager] Sampled acceleration z: " << acc_z_str;

    // log start position difference
    Eigen::Vector3d start_pos_diff = local_data_.position_traj_.evaluateDeBoorT(0.0) - tour[0];
    LOG(INFO) << "[FastPlannerManager] Start position difference: " << start_pos_diff.transpose();

    // log start velocity difference
    Eigen::Vector3d start_vel_diff = local_data_.velocity_traj_.evaluateDeBoorT(0.0) - cur_vel;
    LOG(INFO) << "[FastPlannerManager] Start velocity difference: " << start_vel_diff.transpose();

    // log start acceleration difference
    Eigen::Vector3d start_acc_diff = local_data_.acceleration_traj_.evaluateDeBoorT(0.0) - cur_acc;
    LOG(INFO) << "[FastPlannerManager] Start acceleration difference: "
              << start_acc_diff.transpose();
  }

  double t_total = (ros::Time::now() - t1).toSec();
  ROS_INFO("[FastPlannerManager] Position trajectory optimization time: %lf ms", t_total * 1000);
}

void FastPlannerManager::planExplorationYawWaypointsTraj(
    const double &cur_yaw, const double &cur_yaw_vel, const double &goal_yaw,
    const std::vector<Eigen::Vector3d> &yaw_waypts, const std::vector<int> &yaw_waypts_idx) {
  ros::Time t1 = ros::Time::now();

  const int seg_num = 12;
  double dt_yaw = local_data_.duration_ / seg_num; // time of B-spline segment
  Eigen::Vector3d start_yaw3d(cur_yaw, 0, 0);
  Eigen::Vector3d end_yaw3d(goal_yaw, 0, 0);

  // Yaw traj control points
  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;

  // Initial state
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;
  // Final state
  yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;

  // Call B-spline optimization solver
  int cost_func = BsplineOptimizer::EXPLORATION_YAW_PHASE;

  vector<Eigen::Vector3d> start = {Eigen::Vector3d(start_yaw3d[0], cur_yaw_vel, 0),
                                   Eigen::Vector3d(start_yaw3d[1], 0, 0),
                                   Eigen::Vector3d(start_yaw3d[2], 0, 0)};
  vector<Eigen::Vector3d> end = {Eigen::Vector3d(end_yaw3d[0], 0, 0), Eigen::Vector3d(0, 0, 0)};
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->setWaypoints(yaw_waypts, yaw_waypts_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func);

  // Update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  local_data_.start_yaw_ = local_data_.yaw_traj_.evaluateDeBoorT(0.0)[0];
  local_data_.end_yaw_ = local_data_.yaw_traj_.evaluateDeBoorT(local_data_.duration_)[0];

  local_data_.cur_yaw_ = cur_yaw;
  local_data_.cur_yaw_vel_ = cur_yaw_vel;
  local_data_.goal_yaw_ = goal_yaw;
  local_data_.yaw_waypts_ = yaw_waypts;
  local_data_.yaw_waypts_idx_ = yaw_waypts_idx;

  double t_total = (ros::Time::now() - t1).toSec();
  ROS_INFO("[FastPlannerManager] Yaw trajectory optimization time: %lf s", t_total * 1000);
}

void FastPlannerManager::calcNextYaw(const double &last_yaw, double &yaw) {
  // round yaw to [-PI, PI]
  double round_last = last_yaw;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

bool FastPlannerManager::checkTrajCollision() {
  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoorT(t_now);
  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoorT(t_now + fut_t);
    if (map_server_->getOccupancy(fut_pt) == voxel_mapping::OccupancyType::OCCUPIED) {
      ROS_ERROR("[FastPlannerManager] Collision detected at (%.2f, %.2f, %.2f)", fut_pt.x(),
                fut_pt.y(), fut_pt.z());
      return false;
    }
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

void FastPlannerManager::saveBsplineTraj(const std::string &file_name) {
  std::ofstream file(file_name);
  if (!file.is_open()) {
    ROS_ERROR("[FastPlannerManager] Failed to open bspline file %s", file_name.c_str());
    return;
  }

  // Save bspline parameters to file
  file << "TRAJ_ID: " << local_data_.traj_id_ << std::endl;
  file << "DURATION: " << local_data_.duration_ << std::endl;
  file << "START_POS: " << local_data_.start_pos_.transpose() << std::endl;
  file << "END_POS: " << local_data_.end_pos_.transpose() << std::endl;
  file << "START_YAW: " << local_data_.start_yaw_ << std::endl;
  file << "END_YAW: " << local_data_.end_yaw_ << std::endl;
  file << "TOUR: " << std::endl;
  for (int i = 0; i < local_data_.tour_.size(); ++i) {
    file << local_data_.tour_[i].transpose() << std::endl;
  }
  file << "CUR_VEL: " << local_data_.cur_vel_.transpose() << std::endl;
  file << "CUR_ACC: " << local_data_.cur_acc_.transpose() << std::endl;
  file << "TIME_LB: " << local_data_.time_lb_ << std::endl;
  file << "CUR_YAW: " << local_data_.cur_yaw_ << std::endl;
  file << "CUR_YAW_VEL: " << local_data_.cur_yaw_vel_ << std::endl;
  file << "GOAL_YAW: " << local_data_.goal_yaw_ << std::endl;
  file << "YAW_WAYPTS: " << std::endl;
  for (int i = 0; i < local_data_.yaw_waypts_.size(); ++i) {
    file << local_data_.yaw_waypts_[i].transpose() << std::endl;
  }
  file << "YAW_WAYPTS_IDX: " << std::endl;
  for (int i = 0; i < local_data_.yaw_waypts_idx_.size(); ++i) {
    file << local_data_.yaw_waypts_idx_[i] << std::endl;
  }
  const double dt = 0.02;
  for (double t = 0.0; t < local_data_.duration_; t += dt) {
    Eigen::Vector3d pos = local_data_.position_traj_.evaluateDeBoorT(t);
    Eigen::Vector3d vel = local_data_.velocity_traj_.evaluateDeBoorT(t);
    Eigen::Vector3d acc = local_data_.acceleration_traj_.evaluateDeBoorT(t);
    double yaw = local_data_.yaw_traj_.evaluateDeBoorT(t)[0];

    file << t << " " << pos.x() << " " << pos.y() << " " << pos.z() << " " << vel.x() << " "
         << vel.y() << " " << vel.z() << " " << acc.x() << " " << acc.y() << " " << acc.z() << " "
         << yaw << std::endl;
  }

  file.close();
}

} // namespace fast_planner
