#include <ros/ros.h>

#include "backward/backward.hpp"
#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "perception_utils/perception_utils.h"
#include "polynomial/polynomial_traj.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Int32.h"
#include "trajectory/Bspline.h"
#include "visualization_msgs/Marker.h"

namespace backward {
backward::SignalHandling sh;
}

using fast_planner::NonUniformBspline;
using fast_planner::PerceptionUtils;
using fast_planner::Polynomial;
using fast_planner::PolynomialTraj;

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub, traj_vel_pub;
quadrotor_msgs::PositionCommand cmd;

// Info of generated traj
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

shared_ptr<PerceptionUtils> percep_utils_;

// Info of replan
bool receive_traj_ = false;

bool new_traj_ = false;
bool task_finished_ = false;

// Executed traj, commanded and real ones
vector<Eigen::Vector3d> traj_cmd_, vel_cmd_;

// Data for benchmark comparison
ros::Time flight_start_time, flight_end_time, last_time;
double energy;

double calcPathLength(const vector<Eigen::Vector3d> &path) {
  if (path.empty())
    return 0;
  double len = 0.0;
  for (int i = 0; i < path.size() - 1; ++i) {
    len += (path[i + 1] - path[i]).norm();
  }
  return len;
}

Eigen::Vector4d getColor(const double &h, double alpha) {
  double h1 = h;
  if (h1 < 0.0) {
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;

  if (h1 >= -1e-4 && h1 < 1.0 / 2) {
    lambda = (h1 - 0.0) * 2;
    color1 = Eigen::Vector4d(1, 1, 1, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else {
    lambda = (h1 - 1.0 / 2) * 2;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id = 0) {
  if (traj_pub.getNumSubscribers() == 0)
    return;

  if (path.empty())
    return;

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  // mk.type = visualization_msgs::Marker::LINE_LIST;

  static int traj_mk_id = 0;
  mk.id = traj_mk_id;
  // traj_mk_id++;

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  // smooth path using RC filter
  vector<Eigen::Vector3d> smooth_path;
  smooth_path.push_back(path[0]);
  for (int i = 1; i < path.size(); ++i) {
    double alpha = 0.3;
    Eigen::Vector3d last_pt = smooth_path.back();
    Eigen::Vector3d new_pt = alpha * path[i] + (1 - alpha) * last_pt;
    smooth_path.push_back(new_pt);
  }

  geometry_msgs::Point pt;
  for (int i = 0; i < int(smooth_path.size()); i++) {
    pt.x = smooth_path[i](0);
    pt.y = smooth_path[i](1);
    pt.z = smooth_path[i](2);
    mk.points.push_back(pt);
  }

  traj_pub.publish(mk);
}

void displayTrajVel(const vector<Eigen::Vector3d> positions,
                    const vector<Eigen::Vector3d> velocities) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.points.resize(positions.size());

  for (int i = 0; i < int(positions.size()); i++) {
    cloud.points[i].x = positions[i](0);
    cloud.points[i].y = positions[i](1);
    cloud.points[i].z = positions[i](2);

    double vel_norm = std::min(std::max(velocities[i].norm() / 1.0, 0.0), 1.0);
    Eigen::Vector4d color = getColor(vel_norm, 1.0);
    cloud.points[i].r = color(0) * 255;
    cloud.points[i].g = color(1) * 255;
    cloud.points[i].b = color(2) * 255;
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  traj_vel_pub.publish(cloud_msg);
}

void drawFOV(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_fov";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.02;
  mk.scale.y = 0.02;
  mk.scale.z = 0.02;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  cmd_vis_pub.publish(mk);

  if (list1.size() == 0)
    return;

  // Pub new marker
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;
  cmd_vis_pub.publish(mk);
}

void drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id,
             const Eigen::Vector4d &color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.ns = "current_cmd";
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void replanCallback(std_msgs::Int32 msg) {
  // Informed of new replan, end the current traj after some time
  const double time_out = 0.5;
  ros::Time time_now = ros::Time::now();
  double t_stop;

  if (msg.data == 0) {
    t_stop = (time_now - start_time_).toSec() + time_out;
  } else if (msg.data == 1) {
    ROS_WARN("[TrajServer] Replan type 1: collision detected, stop trajectory immediately");
    t_stop = (time_now - start_time_).toSec();
  } else if (msg.data == 2) {
    ROS_WARN_ONCE("[TrajServer] Replan type 2: exploration finished, stop trajectory immediately");
    t_stop = (time_now - start_time_).toSec();
    task_finished_ = true;
  } else {
    ROS_ERROR("[TrajServer] Invalid replan type: %d", msg.data);
    return;
  }

  traj_duration_ = min(t_stop, traj_duration_);
}

void visCallback(const ros::TimerEvent &e) {
  // Draw the executed traj (desired state)
  displayTrajWithColor(traj_cmd_, 0.1, Eigen::Vector4d(1, 0, 0, 1));
  displayTrajVel(traj_cmd_, vel_cmd_);
}

void bsplineCallback(const trajectory::BsplineConstPtr &msg) {
  // Received traj should have ascending traj_id
  if (msg->traj_id <= traj_id_) {
    ROS_ERROR("[TrajServer] Bspline trajectory ID misordered, incoming: %d, current: %d",
              msg->traj_id, traj_id_);
    return;
  }

  new_traj_ = true;

  ROS_INFO("[TrajServer] Received Bspline trajectory with ID: %d", msg->traj_id);

  // Parse the msg
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }
  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i)
    yaw_pts(i, 0) = msg->yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_.push_back(traj_[2].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;

  // Record the start time of flight
  if (flight_start_time.isZero()) {
    flight_start_time = ros::Time::now();
  }
}

void cmdCallback(const ros::TimerEvent &e) {
  // No publishing before receive traj data
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();
  Eigen::Vector3d pos, vel, acc, jer;
  double yaw, yawdot;

  if (new_traj_) {
    ROS_INFO_STREAM("[TrajServer] t_cur: " << t_cur << ", traj_duration_: " << traj_duration_);
    new_traj_ = false;
  }

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    // Current time within range of planned traj
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    jer = traj_[5].evaluateDeBoorT(t_cur);
  } else if (t_cur >= traj_duration_) {
    // Current time exceed range of planned traj
    // keep publishing the final position and yaw
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = 0.0;
  } else {
    ROS_ERROR("[TrajServer] Invalid time: start_time_: %.2lf, t_cur: %.2lf, traj_duration_: %.2lf",
              start_time_.toSec(), t_cur, traj_duration_);
  }

  // round yaw to [-PI, PI]
  while (yaw < -M_PI) {
    yaw += 2 * M_PI;
  }
  while (yaw > M_PI) {
    yaw -= 2 * M_PI;
  }

  // Report info of the whole flight
  double len = calcPathLength(traj_cmd_);
  double flight_t = (flight_end_time - flight_start_time).toSec();

  // mean vel here is not axis velocity
  ROS_WARN_THROTTLE(1.0, "[TrajServer] Flight time: %.2lf, path length: %.2lf, mean vel: %.2lf",
                    flight_t, len, len / flight_t);

  cmd.header.stamp = time_now;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;
  pos_cmd_pub.publish(cmd);

  // Draw cmd
  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  percep_utils_->setPose(pos, yaw);
  vector<Eigen::Vector3d> l1, l2;
  percep_utils_->getFOV(l1, l2);
  drawFOV(l1, l2);

  // Record info of the executed traj
  if (traj_cmd_.size() == 0) {
    // Add the first position
    traj_cmd_.push_back(pos);
    vel_cmd_.push_back(vel);
  } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
    // Add new different commanded position
    Eigen::Vector3d last_pos = traj_cmd_.back();
    if ((pos - last_pos).norm() > 0.05) {
      ROS_ERROR("[TrajServer] Position trajectory discontinuity detected from %.2lf, %.2lf, %.2lf "
                "to %.2lf, %.2lf, %.2lf with error: %.2lf",
                last_pos[0], last_pos[1], last_pos[2], pos[0], pos[1], pos[2],
                (pos - last_pos).norm());
    }
    traj_cmd_.push_back(pos);
    vel_cmd_.push_back(vel);
    double dt = (time_now - last_time).toSec();
    energy += jer.squaredNorm() * dt;
    flight_end_time = ros::Time::now();
  }
  last_time = time_now;

  // if (traj_cmd_.size() > 100000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 1, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 1, replanCallback);

  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("planning/pos_cmd", 50);
  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);
  traj_vel_pub = node.advertise<sensor_msgs::PointCloud2>("planning/travel_vel_traj", 10);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  ros::NodeHandle nh("~");

  Eigen::Vector3d init_pos, init_pos_offset_body, init_pos_offset;
  double init_yaw;

  // Initial position and yaw
  nh.param("/map_config/init_x", init_pos[0], 0.0);
  nh.param("/map_config/init_y", init_pos[1], 0.0);
  nh.param("/map_config/init_z", init_pos[2], 0.0);
  nh.param("/map_config/init_yaw", init_yaw, 0.0);

  // Initial position offset (body frame)
  nh.param("/traj_server/init_dx", init_pos_offset_body[0], 0.0);
  nh.param("/traj_server/init_dy", init_pos_offset_body[1], 0.0);
  nh.param("/traj_server/init_dz", init_pos_offset_body[2], 0.0);

  // Initial position offset (world frame)
  Eigen::Matrix3d R_init;
  R_init << cos(init_yaw), -sin(init_yaw), 0, sin(init_yaw), cos(init_yaw), 0, 0, 0, 1;
  init_pos_offset = R_init * init_pos_offset_body;

  // Controller parameter
  nh.param("/traj_server/kx_x", cmd.kx[0], -1.0);
  nh.param("/traj_server/kx_y", cmd.kx[1], -1.0);
  nh.param("/traj_server/kx_z", cmd.kx[2], -1.0);
  nh.param("/traj_server/kv_x", cmd.kv[0], -1.0);
  nh.param("/traj_server/kv_y", cmd.kv[1], -1.0);
  nh.param("/traj_server/kv_z", cmd.kv[2], -1.0);

  ROS_INFO("[TrajServer] Initializing");
  ros::Duration(2.0).sleep();

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = init_pos[0];
  cmd.position.y = init_pos[1];
  cmd.position.z = init_pos[2];
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  cmd.yaw = init_yaw;
  cmd.yaw_dot = 0.0;

  percep_utils_.reset(new PerceptionUtils(nh));

  // Initialization for exploration, move to init position
  for (int i = 0; i < 100; ++i) {
    cmd.position.z += init_pos_offset[2] / 100.0;
    pos_cmd_pub.publish(cmd);
    Vector3d pos(cmd.position.x, cmd.position.y, cmd.position.z);
    percep_utils_->setPose(pos, cmd.yaw);
    vector<Eigen::Vector3d> l1, l2;
    percep_utils_->getFOV(l1, l2);
    drawFOV(l1, l2);
    ros::Duration(0.01).sleep();
  }

  for (int i = 0; i < 100; ++i) {
    cmd.position.x += init_pos_offset[0] / 100.0;
    cmd.position.y += init_pos_offset[1] / 100.0;
    pos_cmd_pub.publish(cmd);
    Vector3d pos(cmd.position.x, cmd.position.y, cmd.position.z);
    percep_utils_->setPose(pos, cmd.yaw);
    vector<Eigen::Vector3d> l1, l2;
    percep_utils_->getFOV(l1, l2);
    drawFOV(l1, l2);
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("[TrajServer] Initilization finished");

  ros::Rate rate(100);
  while (ros::ok() && !task_finished_) {
    ros::spinOnce();
    rate.sleep();
  }

  // Flight summary
  double len = calcPathLength(traj_cmd_);
  double flight_t = (flight_end_time - flight_start_time).toSec();
  ROS_WARN("[TrajServer] Flight time: %.2lf, path length: %.2lf, mean vel: %.2lf", flight_t, len,
           len / flight_t);

  ROS_INFO("[TrajServer] Task finished, traj server shutdown");

  return 0;
}
