#include <iostream>
#include <vector>

#include <Eigen/Eigen>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "quadrotor_msgs/PositionCommand.h"

ros::Publisher pub_;
ros::Subscriber sub_;

ros::Publisher pub_marker_array_;
ros::Subscriber sub_marker_array_;

std::vector<Eigen::Vector3d> traj_cmd_, vel_cmd_;
std::vector<double> timestamps_;

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

void displayTrajVel(const std::vector<Eigen::Vector3d> positions,
                    const std::vector<Eigen::Vector3d> velocities) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.points.resize(positions.size());

  for (int i = 0; i < int(positions.size()); i++) {
    cloud.points[i].x = positions[i](0);
    cloud.points[i].y = positions[i](1);
    cloud.points[i].z = positions[i](2);

    double vel_norm = std::min(std::max(velocities[i].norm() / 2.0, 0.0), 1.0);
    Eigen::Vector4d color = getColor(vel_norm, 1.0);
    cloud.points[i].r = color(0) * 255;
    cloud.points[i].g = color(1) * 255;
    cloud.points[i].b = color(2) * 255;
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  pub_.publish(cloud_msg);
}

void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg) {
  ROS_INFO("Received command at %f", msg->header.stamp.toSec());
  Eigen::Vector3d pos_cmd(msg->position.x, msg->position.y, msg->position.z);
  Eigen::Vector3d vel_cmd(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  // enable for nbvp
  if (!traj_cmd_.empty()) {
    vel_cmd = (pos_cmd - traj_cmd_.back()) / (msg->header.stamp.toSec() - timestamps_.back());

    Eigen::Vector3d last_pos = traj_cmd_.back();
    Eigen::Vector3d last_vel = vel_cmd_.back();
    double dist = (pos_cmd - last_pos).norm();
    double vel = vel_cmd.norm();
    double dt = dist / vel;

    int n = 5;
    for (int i = 1; i < n; i++) {
      double alpha = double(i) / double(n);
      traj_cmd_.push_back(last_pos + alpha * (pos_cmd - last_pos));
      vel_cmd_.push_back(last_vel + alpha * (vel_cmd - last_vel));
    }
  }

  timestamps_.push_back(msg->header.stamp.toSec());
  traj_cmd_.push_back(pos_cmd);
  vel_cmd_.push_back(vel_cmd);

  displayTrajVel(traj_cmd_, vel_cmd_);
}

void markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) {
  ROS_INFO("Received marker array at %f", msg->markers[0].header.stamp.toSec());

  // marker array size
  ROS_INFO("Marker array size: %d", int(msg->markers.size()));

  visualization_msgs::MarkerArray markers;
  for (int i = 0; i < int(msg->markers.size() ); i++) {
    ROS_INFO("Marker %d size: %d", i, int(msg->markers[i].points.size()));
    visualization_msgs::Marker marker = msg->markers[i];

    // change color alpha to 0.4
    for (int j = 0; j < int(marker.colors.size()); j++) {
      marker.colors[j].a = 0.2;
    }

    markers.markers.push_back(marker);

  }

  // for (int i = 0; i < int(msg->markers.size() - 2); i++) {
  //   visualization_msgs::Marker marker = msg->markers[i];
  //   markers.markers.push_back(marker);
  // }

  // // check cube list height, keep only cube with pos z < 1.5
  // visualization_msgs::Marker marker = msg->markers[msg->markers.size() - 1];
  // marker.points.clear();
  // marker.colors.clear();
  // for (int i = 0; i < int(msg->markers[msg->markers.size() - 1].points.size()); i++) {
  //   // ROS_INFO("Point %d z: %f", i, msg->markers[msg->markers.size() - 1].points[i].z);
  //   if (msg->markers[msg->markers.size() - 1].points[i].z < 1.5) {
  //     marker.points.push_back(msg->markers[msg->markers.size() - 1].points[i]);
  //     marker.colors.push_back(msg->markers[msg->markers.size() - 1].colors[i]);
  //   }
  // }
  // ROS_INFO("Filtered Marker %d size: %d", int(msg->markers.size() - 1), int(marker.points.size()));
  // markers.markers.push_back(marker);

  // marker = msg->markers[msg->markers.size() - 2];
  // marker.points.clear();
  // marker.colors.clear();
  // for (int i = 0; i < int(msg->markers[msg->markers.size() - 2].points.size()); i++) {
  //   // ROS_INFO("Point %d z: %f", i, msg->markers[msg->markers.size() - 2].points[i].z);
  //   if (msg->markers[msg->markers.size() - 2].points[i].z < 1.5) {
  //     marker.points.push_back(msg->markers[msg->markers.size() - 2].points[i]);
  //     marker.colors.push_back(msg->markers[msg->markers.size() - 2].colors[i]);
  //   }
  // }
  // ROS_INFO("Filtered Marker %d size: %d", int(msg->markers.size() - 2), int(marker.points.size()));

  pub_marker_array_.publish(markers);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_visualizer");
  ros::NodeHandle nh("~");

  pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planning/travel_vel_traj", 1);
  sub_ = nh.subscribe("/planning/pos_cmd", 1, cmdCallback);

  // pub_marker_array_ =
  //     nh.advertise<visualization_msgs::MarkerArray>("/firefly/nbvPlanner/octomap_occupied2", 1);
  // sub_marker_array_ = nh.subscribe("/firefly/nbvPlanner/octomap_occupied", 1, markerArrayCallback);

  ros::Duration(0.1).sleep();

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
