#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <ros/ros.h>

#include "bspline/bspline_optimizer.h"
#include "bspline/non_uniform_bspline.h"
#include "pathfinding/astar.h"
#include "polynomial/polynomial_traj.h"
#include "voxel_mapping/map_server.h"

using namespace voxel_mapping;

namespace fast_planner {
struct PlanParameters {
  double max_vel_, max_acc_; // physical limits
  double ctrl_pt_dist_;       // distance between adjacient B-spline control points
};

struct LocalTrajData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int traj_id_;
  double duration_;
  ros::Time start_time_;
  Eigen::Vector3d start_pos_, end_pos_;
  double start_yaw_, end_yaw_;

  PolynomialTraj init_traj_poly_;
  NonUniformBspline init_traj_bspline_;
  NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_;

  PolynomialTraj yaw_init_traj_poly_;
  NonUniformBspline yaw_init_traj_bspline_;
  NonUniformBspline yaw_traj_, yawdot_traj_, yawdotdot_traj_;

  // Bspline optimization input
  vector<Eigen::Vector3d> tour_;
  Eigen::Vector3d cur_vel_, cur_acc_;
  double time_lb_;
  double cur_yaw_, cur_yaw_vel_, goal_yaw_;
  std::vector<Eigen::Vector3d> yaw_waypts_;
  std::vector<int> yaw_waypts_idx_;
};

class FastPlannerManager {
public:
  typedef shared_ptr<FastPlannerManager> Ptr;
  typedef shared_ptr<const FastPlannerManager> ConstPtr;

  FastPlannerManager() {}
  ~FastPlannerManager() {}

  void init(ros::NodeHandle &nh);
  void planExplorationPositionTraj(const vector<Eigen::Vector3d> &tour,
                                   const Eigen::Vector3d &cur_vel, const Eigen::Vector3d &cur_acc,
                                   const double &time_lb, const bool verbose = false);
  void planExplorationYawWaypointsTraj(const double &cur_yaw, const double &cur_yaw_vel,
                                       const double &goal_yaw,
                                       const std::vector<Eigen::Vector3d> &yaw_waypts,
                                       const std::vector<int> &yaw_waypts_idx);
  bool checkTrajCollision();

  // Save traj_id, duration, evaluated position, velocity, acceleration
  void saveBsplineTraj(const std::string &file_name);

  PlanParameters pp_;
  LocalTrajData local_data_;
  Astar::Ptr path_finder_;
  RayCaster::Ptr caster_;
  MapServer::Ptr map_server_;

private:
  void calcNextYaw(const double &last_yaw, double &yaw);
  
  std::vector<BsplineOptimizer::Ptr> bspline_optimizers_;
};
} // namespace fast_planner

#endif