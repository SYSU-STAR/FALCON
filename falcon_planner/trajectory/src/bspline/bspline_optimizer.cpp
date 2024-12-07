#include "bspline/bspline_optimizer.h"

namespace fast_planner {
const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
const int BsplineOptimizer::DISTANCE = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::START = (1 << 3);
const int BsplineOptimizer::END = (1 << 4);
const int BsplineOptimizer::WAYPOINTS = (1 << 5);
const int BsplineOptimizer::MINTIME = (1 << 6);

const int BsplineOptimizer::EXPLORATION_POSITION_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY |
    BsplineOptimizer::START | BsplineOptimizer::END | BsplineOptimizer::MINTIME;
const int BsplineOptimizer::EXPLORATION_YAW_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START | BsplineOptimizer::END |
    BsplineOptimizer::WAYPOINTS;

void BsplineOptimizer::setParam(ros::NodeHandle &nh) {
  nh.param("/bspline_opt/pos/smoothness", pos_smooth_, -1.0);
  nh.param("/bspline_opt/pos/distance", pos_dist_, -1.0);
  nh.param("/bspline_opt/pos/feasibility", pos_feasi_, -1.0);
  nh.param("/bspline_opt/pos/start", pos_start_, -1.0);
  nh.param("/bspline_opt/pos/end", pos_end_, -1.0);
  nh.param("/bspline_opt/pos/time", pos_time_, -1.0);

  nh.param("/bspline_opt/yaw/smoothness", yaw_smooth_, -1.0);
  nh.param("/bspline_opt/yaw/start", yaw_start_, -1.0);
  nh.param("/bspline_opt/yaw/end", yaw_end_, -1.0);
  nh.param("/bspline_opt/yaw/waypoint", yaw_waypt_, -1.0);

  nh.param("/bspline_opt/bspline_degree", bspline_degree_, 3);
  nh.param("/bspline_opt/safe_distance", safe_distance_, -1.0);
  nh.param("/bspline_opt/max_iteration_num", max_iteration_num_, -1);
  nh.param("/bspline_opt/max_iteration_time", max_iteration_time_, -1.0);

  if (false) {
    std::cout << "pos_smooth: " << pos_smooth_ << std::endl;
    std::cout << "pos_dist: " << pos_dist_ << std::endl;
    std::cout << "pos_feasi: " << pos_feasi_ << std::endl;
    std::cout << "pos_start: " << pos_start_ << std::endl;
    std::cout << "pos_end: " << pos_end_ << std::endl;
    std::cout << "pos_time: " << pos_time_ << std::endl;

    std::cout << "yaw_smooth: " << yaw_smooth_ << std::endl;
    std::cout << "yaw_start: " << yaw_start_ << std::endl;
    std::cout << "yaw_end: " << yaw_end_ << std::endl;
    std::cout << "yaw_waypt: " << yaw_waypt_ << std::endl;

    std::cout << "bspline_degree: " << bspline_degree_ << std::endl;
    std::cout << "safe_distance: " << safe_distance_ << std::endl;
    std::cout << "max_iteration_num: " << max_iteration_num_ << std::endl;
    std::cout << "max_iteration_time: " << max_iteration_time_ << std::endl;
  }

  time_lb_ = -1; // Not used by in most case
}

void BsplineOptimizer::setMap(const MapServer::Ptr &map_server) { this->map_server_ = map_server; }

void BsplineOptimizer::setCostFunction(const int &cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS)
    cost_str += " SMOOTHNESS |";
  if (cost_function_ & DISTANCE)
    cost_str += " DISTANCE |";
  if (cost_function_ & FEASIBILITY)
    cost_str += " FEASIBILITY |";
  if (cost_function_ & START)
    cost_str += " START |";
  if (cost_function_ & END)
    cost_str += " END |";
  if (cost_function_ & WAYPOINTS)
    cost_str += " WAYPOINTS |";
  if (cost_function_ & MINTIME)
    cost_str += " MINTIME |";

  // ROS_INFO_STREAM("[BsplineOptimizer] cost func:" << cost_str);
}

void BsplineOptimizer::setBoundaryStates(const vector<Eigen::Vector3d> &start,
                                         const vector<Eigen::Vector3d> &end) {
  start_state_ = start;
  end_state_ = end;
}

void BsplineOptimizer::setTimeLowerBound(const double &lb) { time_lb_ = lb; }

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d> &waypts,
                                    const vector<int> &waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

void BsplineOptimizer::setPhysicalLimits(const double &max_vel, const double &max_acc) {
  max_vel_ = max_vel;
  max_acc_ = max_acc;
}

void BsplineOptimizer::optimize(Eigen::MatrixXd &points, double &dt, const int &cost_function) {
  if (start_state_.empty()) {
    ROS_ERROR("Initial state undefined!");
    return;
  }
  control_points_ = points;
  knot_span_ = dt;
  setCostFunction(cost_function);

  // Set necessary data and flag
  dim_ = control_points_.cols();
  if (dim_ == 1)
    order_ = 3;
  else
    order_ = bspline_degree_;
  point_num_ = control_points_.rows();
  optimize_time_ = cost_function_ & MINTIME;
  variable_num_ = optimize_time_ ? dim_ * point_num_ + 1 : dim_ * point_num_;
  if (variable_num_ <= 0) {
    ROS_ERROR("Empty varibale to optimization solver.");
    return;
  }

  // ROS_ERROR("point_num_: %d, variable_num_: %d", point_num_, variable_num_);
  // ROS_ERROR("variable_num_: %d", variable_num_);

  pt_dist_ = 0.0;
  for (int i = 0; i < control_points_.rows() - 1; ++i) {
    pt_dist_ += (control_points_.row(i + 1) - control_points_.row(i)).norm();
  }
  pt_dist_ /= double(point_num_);

  min_cost_ = std::numeric_limits<double>::max();
  g_q_.resize(point_num_);
  g_smoothness_.resize(point_num_);
  g_distance_.resize(point_num_);
  g_feasibility_.resize(point_num_);
  g_start_.resize(point_num_);
  g_end_.resize(point_num_);
  g_waypoints_.resize(point_num_);
  g_time_.resize(point_num_);

  optimize();

  points = control_points_;
  dt = knot_span_;
  start_state_.clear();
  time_lb_ = -1;
}

void BsplineOptimizer::optimize() {
  // Optimize all control points and maybe knot span dt
  // Use NLopt solver
  // G/L denotes global/local optimization and N/D denotes derivative-free/gradient-based algorithms
  nlopt::opt opt(nlopt::algorithm::LD_LBFGS, variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_);
  opt.set_maxtime(max_iteration_time_);
  opt.set_xtol_rel(1e-4);

  // Set axis aligned bounding box for optimization
  Eigen::Vector3d bmin, bmax;
  map_server_->getBox(bmin, bmax);
  for (int k = 0; k < 3; ++k) {
    bmin[k] += 0.1;
    bmax[k] -= 0.1;
  }

  vector<double> q(variable_num_);
  // Variables for control points
  for (int i = 0; i < point_num_; ++i)
    for (int j = 0; j < dim_; ++j) {
      double cij = control_points_(i, j);
      if (dim_ != 1)
        cij = max(min(cij, bmax[j % 3]), bmin[j % 3]);
      q[dim_ * i + j] = cij;
    }
  // Variables for knot span
  if (optimize_time_)
    q[variable_num_ - 1] = knot_span_;

  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double bound = 10.0;
    for (int i = 0; i < 3 * point_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
      lb[i] = max(lb[i], bmin[i % 3]);
      ub[i] = min(ub[i], bmax[i % 3]);
    }
    if (optimize_time_) {
      lb[variable_num_ - 1] = 0.0;
      ub[variable_num_ - 1] = 5.0;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  auto t1 = ros::Time::now();
  try {
    double final_cost;
    nlopt::result result = opt.optimize(q, final_cost);
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
  }

  for (int i = 0; i < point_num_; ++i)
    for (int j = 0; j < dim_; ++j)
      control_points_(i, j) = best_variable_[dim_ * i + j];
  if (optimize_time_)
    knot_span_ = best_variable_[variable_num_ - 1];

  // Print final costs
  // if (true) {
  if (false) {
    // if (final_costs_.size() == 6) {
    ROS_INFO("[BsplineOptimizer] Final costs:");
    if (final_costs_.size() == 6) {
      // Position phase
      std::vector<std::string> cost_names = {"Smoothness", "Distance", "Feasibility",
                                             "Start",      "End",      "Time"};
      for (int i = 0; i < final_costs_.size(); ++i) {
        ROS_INFO_STREAM("[BsplineOptimizer] " << cost_names[i] << ": " << final_costs_[i]);
      }
    } else if (final_costs_.size() == 4) {
      std::vector<std::string> cost_names = {"Smoothness", "Start", "End", "Waypoints"};
      // Yaw phase
      for (int i = 0; i < final_costs_.size(); ++i) {
        ROS_INFO_STREAM("[BsplineOptimizer] " << cost_names[i] << ": " << final_costs_[i]);
      }
    }
  }
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                          double &cost, vector<Eigen::Vector3d> &gradient_q,
                                          double &gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - 3; i++) {
    /* evaluate jerk */
    // Test jerk cost
    // 3-rd order derivative = 1/(ts)^3*(q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1]
    // - q[i])
    Eigen::Vector3d ji = (q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i]) / pt_dist_;
    cost += ji.squaredNorm();
    temp_j = 2 * ji / pt_dist_;

    // Deprecated version
    // jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    // cost += jerk.squaredNorm();
    // temp_j = 2.0 * jerk;

    /* jerk gradient_q */
    // d cost / d q[i] = d cost / d jerk * d jerk / d q[i]
    // gradient_q = d cost / d q[i] for each i
    gradient_q[i + 0] += -temp_j;
    gradient_q[i + 1] += 3.0 * temp_j;
    gradient_q[i + 2] += -3.0 * temp_j;
    gradient_q[i + 3] += temp_j;
    // if (optimize_time_)
    //   gt += -6 * ji.dot(ji) / dt;
  }
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d> &q, double &cost,
                                        vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);
  for (int i = 0; i < q.size(); i++) {
    map_server_->getESDF()->getDistanceAndGradient(q[i], dist, dist_grad);
    if (dist_grad.norm() > 1e-4)
      dist_grad.normalize();

    if (dist < safe_distance_) {
      cost += pow(dist - safe_distance_, 2);
      gradient_q[i] += 2.0 * (dist - safe_distance_) * dist_grad;
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                           double &cost, vector<Eigen::Vector3d> &gradient_q,
                                           double &gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  gt = 0.0;

  // Abbreviation of params
  const double dt_inv = 1 / dt;
  const double dt_inv2 = dt_inv * dt_inv;

  for (int i = 0; i < q.size() - 1; ++i) {
    // Control point of velocity
    Eigen::Vector3d vi = (q[i + 1] - q[i]) * dt_inv;
    for (int k = 0; k < 3; ++k) {
      // Calculate cost for each axis
      double vd = fabs(vi[k]) - max_vel_;
      if (vd > 0.0) {
        cost += pow(vd, 2);
        double sign = vi[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * vd * sign * dt_inv;
        gradient_q[i][k] += -tmp;
        gradient_q[i + 1][k] += tmp;
        if (optimize_time_)
          gt += tmp * (-vi[k]);
      }
    }
  }

  // Acc feasibility cost
  for (int i = 0; i < q.size() - 2; ++i) {
    Eigen::Vector3d ai = (q[i + 2] - 2 * q[i + 1] + q[i]) * dt_inv2;
    for (int k = 0; k < 3; ++k) {
      double ad = fabs(ai[k]) - max_acc_;
      if (ad > 0.0) {
        cost += pow(ad, 2);
        double sign = ai[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * ad * sign * dt_inv2;
        gradient_q[i][k] += tmp;
        gradient_q[i + 1][k] += -2 * tmp;
        gradient_q[i + 2][k] += tmp;
        if (optimize_time_)
          gt += tmp * ai[k] * (-2) * dt;
      }
    }
  }
}

void BsplineOptimizer::calcStartCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                     double &cost, vector<Eigen::Vector3d> &gradient_q,
                                     double &gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  // std::fill(gradient_q.begin(), gradient_q.end(), zero);
  for (int i = 0; i < 3; ++i)
    gradient_q[i] = zero;
  gt = 0.0;

  Eigen::Vector3d q1, q2, q3, dq;
  q1 = q[0];
  q2 = q[1];
  q3 = q[2];

  // Start position
  static const double w_pos = 10.0;
  dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - start_state_[0];
  cost += w_pos * dq.squaredNorm();
  gradient_q[0] += w_pos * 2 * dq * (1 / 6.0);
  gradient_q[1] += w_pos * 2 * dq * (4 / 6.0);
  gradient_q[2] += w_pos * 2 * dq * (1 / 6.0);

  // Start velocity
  dq = 1 / (2 * dt) * (q3 - q1) - start_state_[1];
  cost += dq.squaredNorm();
  gradient_q[0] += 2 * dq * (-1.0) / (2 * dt);
  gradient_q[2] += 2 * dq * 1.0 / (2 * dt);
  if (optimize_time_)
    gt += dq.dot(q3 - q1) / (-dt * dt);

  // Start acceleration
  dq = 1 / (dt * dt) * (q1 - 2 * q2 + q3) - start_state_[2];
  cost += dq.squaredNorm();
  gradient_q[0] += 2 * dq * 1.0 / (dt * dt);
  gradient_q[1] += 2 * dq * (-2.0) / (dt * dt);
  gradient_q[2] += 2 * dq * 1.0 / (dt * dt);
  if (optimize_time_)
    gt += dq.dot(q1 - 2 * q2 + q3) / (-dt * dt * dt);
}

void BsplineOptimizer::calcEndCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                                   vector<Eigen::Vector3d> &gradient_q, double &gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  // std::fill(gradient_q.begin(), gradient_q.end(), zero);
  for (int i = q.size() - 3; i < q.size(); ++i)
    gradient_q[i] = zero;
  gt = 0.0;

  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  // End position
  dq = 1 / 6.0 * (q_1 + 4 * q_2 + q_3) - end_state_[0];
  cost += dq.squaredNorm();
  gradient_q[q.size() - 1] += 2 * dq * (1 / 6.0);
  gradient_q[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient_q[q.size() - 3] += 2 * dq * (1 / 6.0);

  if (end_state_.size() >= 2) {
    // End velocity
    dq = 1 / (2 * dt) * (q_1 - q_3) - end_state_[1];
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (2 * dt);
    gradient_q[q.size() - 3] += 2 * dq * (-1.0) / (2 * dt);
    if (optimize_time_)
      gt += dq.dot(q_1 - q_3) / (-dt * dt);
  }
  if (end_state_.size() == 3) {
    // End acceleration
    dq = 1 / (dt * dt) * (q_1 - 2 * q_2 + q_3) - end_state_[2];
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (dt * dt);
    gradient_q[q.size() - 2] += 2 * dq * (-2.0) / (dt * dt);
    gradient_q[q.size() - 3] += 2 * dq * 1.0 / (dt * dt);
    if (optimize_time_)
      gt += dq.dot(q_1 - 2 * q_2 + q_3) / (-dt * dt * dt);
  }
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d> &q, double &cost,
                                         vector<Eigen::Vector3d> &gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int idx = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient_q[idx] += dq * (2.0 / 6.0);     // 2*dq*(1/6)
    gradient_q[idx + 1] += dq * (8.0 / 6.0); // 2*dq*(4/6)
    gradient_q[idx + 2] += dq * (2.0 / 6.0);
  }
}

void BsplineOptimizer::calcTimeCost(const double &dt, double &cost, double &gt) {
  // Min time
  double duration = (point_num_ - order_) * dt;
  cost = duration;
  gt = double(point_num_ - order_);

  // Time lower bound
  if (time_lb_ > 0 && duration < time_lb_) {
    static const double w_lb = 10;
    cost += w_lb * pow(duration - time_lb_, 2);
    gt += w_lb * 2 * (duration - time_lb_) * (point_num_ - order_);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double> &x, std::vector<double> &grad,
                                   double &f_combine) {
  ros::Time t1 = ros::Time::now();

  for (int i = 0; i < point_num_; ++i) {
    for (int j = 0; j < dim_; ++j)
      g_q_[i][j] = x[dim_ * i + j];
    for (int j = dim_; j < 3; ++j)
      g_q_[i][j] = 0.0;
  }
  const double dt = optimize_time_ ? x[variable_num_ - 1] : knot_span_;

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  bool verbose = false;
  double f_smoothness = 0.0;
  double gt_smoothness = 0.0;
  double f_distance = 0.0;
  double f_feasibility = 0.0;
  double gt_feasibility = 0.0;
  double f_start = 0.0;
  double gt_start = 0.0;
  double f_end = 0.0;
  double gt_end = 0.0;
  double f_waypoints = 0.0;
  double f_time = 0.0;
  double gt_time = 0.0;

  double ld_smooth, ld_dist, ld_feasi, ld_start, ld_end, ld_waypt, ld_time;

  if (cost_function_ == EXPLORATION_POSITION_PHASE) {
    ld_smooth = pos_smooth_;
    ld_dist = pos_dist_;
    ld_feasi = pos_feasi_;
    ld_start = pos_start_;
    ld_end = pos_end_;
    ld_time = pos_time_;
  } else if (cost_function_ == EXPLORATION_YAW_PHASE) {
    ld_smooth = yaw_smooth_;
    ld_start = yaw_start_;
    ld_end = yaw_end_;
    ld_waypt = yaw_waypt_;
  } else {
    CHECK(false) << "Invalid cost function.";
    return;
  }

  if (cost_function_ & SMOOTHNESS) {
    // double f_smoothness = 0.0, gt_smoothness = 0.0;
    calcSmoothnessCost(g_q_, dt, f_smoothness, g_smoothness_, gt_smoothness);
    f_combine += ld_smooth * f_smoothness;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_smooth * g_smoothness_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_smooth * gt_smoothness;
  }
  if (cost_function_ & DISTANCE) {
    // double f_distance = 0.0;
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += ld_dist * f_distance;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_dist * g_distance_[i](j);
  }
  if (cost_function_ & FEASIBILITY) {
    // double f_feasibility = 0.0, gt_feasibility = 0.0;
    calcFeasibilityCost(g_q_, dt, f_feasibility, g_feasibility_, gt_feasibility);
    f_combine += ld_feasi * f_feasibility;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_feasi * g_feasibility_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_feasi * gt_feasibility;
  }
  if (cost_function_ & START) {
    // double f_start = 0.0, gt_start = 0.0;
    calcStartCost(g_q_, dt, f_start, g_start_, gt_start);
    f_combine += ld_start * f_start;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_start * g_start_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_start * gt_start;
  }
  if (cost_function_ & END) {
    // double f_end = 0.0, gt_end = 0.0;
    calcEndCost(g_q_, dt, f_end, g_end_, gt_end);
    f_combine += ld_end * f_end;
    for (int i = point_num_ - 3; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_end * g_end_[i](j);
    if (optimize_time_)
      grad[variable_num_ - 1] += ld_end * gt_end;
  }
  if (cost_function_ & WAYPOINTS) {
    // double f_waypoints = 0.0;
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += ld_waypt * f_waypoints;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_waypt * g_waypoints_[i](j);
  }
  if (cost_function_ & MINTIME) {
    // double f_time = 0.0, gt_time = 0.0;
    calcTimeCost(dt, f_time, gt_time);
    f_combine += ld_time * f_time;
    grad[variable_num_ - 1] += ld_time * gt_time;
  }

  if (false) {
    if (cost_function_ & SMOOTHNESS)
      ROS_INFO("Smoothness cost:      %f", ld_smooth * f_smoothness);
    if (cost_function_ & DISTANCE)
      ROS_INFO("Distance cost:        %f", ld_dist * f_distance);
    if (cost_function_ & FEASIBILITY)
      ROS_INFO("Feasibility cost:     %f", ld_feasi * f_feasibility);
    if (cost_function_ & START)
      ROS_INFO("Start cost:           %f", ld_start * f_start);
    if (cost_function_ & END)
      ROS_INFO("End cost:             %f", ld_end * f_end);
    if (cost_function_ & WAYPOINTS)
      ROS_INFO("Waypoint cost:        %f", ld_waypt * f_waypoints);
    if (cost_function_ & MINTIME)
      ROS_INFO("Time cost:            %f", ld_time * f_time);
    ROS_INFO("TOTAL:                %f", f_combine);
  }

  // Record final costs
  final_costs_.clear();
  if (cost_function_ & SMOOTHNESS)
    final_costs_.push_back(ld_smooth * f_smoothness);
  if (cost_function_ & DISTANCE)
    final_costs_.push_back(ld_dist * f_distance);
  if (cost_function_ & FEASIBILITY)
    final_costs_.push_back(ld_feasi * f_feasibility);
  if (cost_function_ & START)
    final_costs_.push_back(ld_start * f_start);
  if (cost_function_ & END)
    final_costs_.push_back(ld_end * f_end);
  if (cost_function_ & WAYPOINTS)
    final_costs_.push_back(ld_waypt * f_waypoints);
  if (cost_function_ & MINTIME)
    final_costs_.push_back(ld_time * f_time);
}

double BsplineOptimizer::costFunction(const std::vector<double> &x, std::vector<double> &grad,
                                      void *func_data) {
  BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
  double cost;
  opt->combineCost(x, grad, cost);

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_ = cost;
    opt->best_variable_ = x;
  }
  return cost;
}
} // namespace fast_planner