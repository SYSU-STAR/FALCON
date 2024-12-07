#include "traj_utils/planning_visualization.h"

using std::cout;
using std::endl;
namespace fast_planner {
PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh) {
  node = nh;

  traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 100);
  pubs_.push_back(traj_pub_);

  topo_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/topo_path", 100);
  pubs_.push_back(topo_pub_);

  predict_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/prediction", 100);
  pubs_.push_back(predict_pub_);

  visib_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/"
                                                          "visib_constraint",
                                                          100);
  pubs_.push_back(visib_pub_);

  frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 1000);
  pubs_.push_back(frontier_pub_);

  yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 100);
  pubs_.push_back(yaw_pub_);

  viewpoint_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/viewpoints", 1000);
  pubs_.push_back(viewpoint_pub_);

  hgrid_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/hgrid", 1000);
  pubs_.push_back(hgrid_pub_);

  debug_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/debug", 1000);
  pubs_.push_back(debug_pub_);

  pose_pub_ = node.advertise<geometry_msgs::PoseStamped>("/planning_vis/pose", 1000);
  pubs_.push_back(pose_pub_);

  pose_array_pub_ = node.advertise<geometry_msgs::PoseArray>("/planning_vis/pose_array", 1000);
  pubs_.push_back(pose_array_pub_);

  frontier_pcl_pub_ = node.advertise<sensor_msgs::PointCloud2>("/planning_vis/frontier_pcl", 1000);
  dormant_frontier_pcl_pub_ =
      node.advertise<sensor_msgs::PointCloud2>("/planning_vis/dormant_frontier_pcl", 1000);
  tiny_frontier_pcl_pub_ =
      node.advertise<sensor_msgs::PointCloud2>("/planning_vis/tiny_frontier_pcl", 1000);
  trajectory_with_velocity_pub_ =
      node.advertise<sensor_msgs::PointCloud2>("/planning_vis/trajectory_with_velocity", 1000);

  node.param("planning_vis/world_frame", world_frame_, string("world"));

  last_topo_path1_num_ = 0;
  last_topo_path2_num_ = 0;
  last_bspline_phase1_num_ = 0;
  last_bspline_phase2_num_ = 0;
  last_frontier_num_ = 0;
}

void PlanningVisualization::fillBasicInfo(visualization_msgs::Marker &mk,
                                          const Eigen::Vector3d &scale,
                                          const Eigen::Vector4d &color, const string &ns,
                                          const int &id, const int &shape) {
  mk.header.frame_id = world_frame_;
  mk.header.stamp = ros::Time::now();
  mk.id = id;
  mk.ns = ns;
  mk.type = shape;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = scale[0];
  mk.scale.y = scale[1];
  mk.scale.z = scale[2];
}

void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker &mk,
                                             const vector<Eigen::Vector3d> &list) {
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
}

void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker &mk,
                                             const vector<Eigen::Vector3d> &list1,
                                             const vector<Eigen::Vector3d> &list2) {
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
}

void PlanningVisualization::drawBox(const Eigen::Vector3d &center, const Eigen::Vector3d &scale,
                                    const Eigen::Vector4d &color, const string &ns, const int &id,
                                    const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::Marker::CUBE);
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  mk.pose.position.x = center[0];
  mk.pose.position.y = center[1];
  mk.pose.position.z = center[2];
  mk.action = visualization_msgs::Marker::ADD;

  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawText(const Eigen::Vector3d &pos, const string &text,
                                     const double &scale, const Eigen::Vector4d &color,
                                     const string &ns, const int &id, const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::TEXT_VIEW_FACING);

  // // clean old marker
  // mk.action = visualization_msgs::Marker::DELETE;
  // pubs_[pub_id].publish(mk);

  // pub new marker
  mk.text = text;
  mk.pose.position.x = pos[0];
  mk.pose.position.y = pos[1];
  mk.pose.position.z = pos[2];
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::removeText(const string &ns, const int &id, const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d::Zero(), Color::Black(), ns, id,
                visualization_msgs::Marker::TEXT_VIEW_FACING);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);
}

void PlanningVisualization::drawSpheres(const vector<Eigen::Vector3d> &list, const double &scale,
                                        const Eigen::Vector4d &color, const string &ns,
                                        const int &id, const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::SPHERE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawCubes(const vector<Eigen::Vector3d> &list, const double &scale,
                                      const Eigen::Vector4d &color, const string &ns, const int &id,
                                      const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::CUBE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d> &list1,
                                      const vector<Eigen::Vector3d> &list2, const double &scale,
                                      const Eigen::Vector4d &color, const string &ns, const int &id,
                                      const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  if (list1.empty() || list2.empty() || list1.size() != list2.size()) {
    // // ROS_WARN error details
    // ROS_WARN("drawLines error @ %s", ns.c_str());
    // if (list1.empty())
    //   ROS_WARN("list1 is empty");
    // if (list2.empty())
    //   ROS_WARN("list2 is empty");
    // if (list1.size() != list2.size())
    //   ROS_WARN("list1 and list2 are not the same size. list1: %d, list2: %d", int(list1.size()),
    //            int(list2.size()));
    return;
  }

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list1.size() == 0)
    return;

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d> &list, const double &scale,
                                      const Eigen::Vector4d &color, const string &ns, const int &id,
                                      const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list.size() == 0)
    return;

  // split the single list into two
  vector<Eigen::Vector3d> list1, list2;
  for (int i = 0; i < list.size() - 1; ++i) {
    list1.push_back(list[i]);
    list2.push_back(list[i + 1]);
  }

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawLinesList(const vector<vector<Eigen::Vector3d>> &lines_list,
                                          const double &scale, const string &ns,
                                          const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  vector<Eigen::Vector4d> colors = {Color::Red(),    Color::Green(), Color::Blue(), Color::Yellow(),
                                    Color::Purple(), Color::Cyan(),  Color::White()};
  int id = 0;
  double height_offset = 1.0;

  static int last_lines_list_size = 0;

  if (last_lines_list_size > lines_list.size()) {
    for (int i = lines_list.size(); i < last_lines_list_size; ++i) {
      visualization_msgs::Marker mk;
      fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), colors[id], ns, i,
                    visualization_msgs::Marker::LINE_LIST);
      mk.action = visualization_msgs::Marker::DELETE;
      pubs_[pub_id].publish(mk);
    }
  }

  for (const auto &list : lines_list) {
    if (list.size() == 0)
      continue;

    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), colors[id % colors.size()], ns, id,
                  visualization_msgs::Marker::LINE_LIST);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    pubs_[pub_id].publish(mk);

    // pub new marker
    vector<Eigen::Vector3d> list1, list2;
    for (int i = 0; i < list.size() - 1; ++i) {
      list1.push_back(list[i] + Eigen::Vector3d(0, 0, height_offset));
      list2.push_back(list[i + 1] + Eigen::Vector3d(0, 0, height_offset));
    }

    fillGeometryInfo(mk, list1, list2);
    mk.action = visualization_msgs::Marker::ADD;
    pubs_[pub_id].publish(mk);

    id += 1;
    height_offset += 1.0;
  }

  last_lines_list_size = lines_list.size();
}

void PlanningVisualization::drawMidArrowLines(const vector<Eigen::Vector3d> &list,
                                              const double &scale, const Eigen::Vector4d &color,
                                              const string &ns, const int &id, const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list.size() == 0)
    return;

  // split the single list into two
  vector<Eigen::Vector3d> list1, list2;
  for (int i = 0; i < list.size() - 1; ++i) {
    list1.push_back(list[i]);
    list2.push_back(list[i + 1]);

    // Add an arrow at the middle of each line, two lines from midpoint, angle 45, len 0.5m
    Eigen::Vector3d mid = (list[i] + list[i + 1]) / 2;
    Eigen::Vector3d dir = (list[i] - list[i + 1]).normalized();
    Position p1, p2;

    double alpah1 = 30.0 / 180.0 * M_PI;
    p1[0] = dir[0] * cos(alpah1) - dir[1] * sin(alpah1);
    p1[1] = dir[0] * sin(alpah1) + dir[1] * cos(alpah1);
    p1[2] = 0.0;

    double alpah2 = -30.0 / 180.0 * M_PI;
    p2[0] = dir[0] * cos(alpah2) - dir[1] * sin(alpah2);
    p2[1] = dir[0] * sin(alpah2) + dir[1] * cos(alpah2);
    p2[2] = 0.0;

    p1 = p1 * 0.3 + mid;
    p2 = p2 * 0.3 + mid;

    list1.push_back(mid);
    list2.push_back(p1);
    list1.push_back(mid);
    list2.push_back(p2);
  }

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d> &list,
                                              double resolution, const Eigen::Vector4d &color,
                                              int id, int pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  mk.header.frame_id = world_frame_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

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

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d> &list, double resolution,
                                            const Eigen::Vector4d &color, int id, int pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  mk.header.frame_id = world_frame_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

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

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d> &list1,
                                            const vector<Eigen::Vector3d> &list2, double line_width,
                                            const Eigen::Vector4d &color, int id, int pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  mk.header.frame_id = world_frame_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

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
  pubs_[pub_id].publish(mk);

  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawPose(const Position &position,
                                     const Eigen::Quaterniond &orientation, const string &ns,
                                     const int &id) {
  if (pubs_[PlanningVisualization::PUBLISHER::POSE].getNumSubscribers() == 0)
    return;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = world_frame_;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
  pose.pose.orientation.w = orientation.w();

  pubs_[PlanningVisualization::PUBLISHER::POSE].publish(pose);
}

void PlanningVisualization::drawPoses(const vector<pair<Position, Eigen::Quaterniond>> &poses,
                                      const string &ns) {
  if (pubs_[PlanningVisualization::PUBLISHER::POSE_ARRAY].getNumSubscribers() == 0)
    return;

  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = world_frame_;
  pose_array.header.stamp = ros::Time::now();

  for (const auto &pose : poses) {
    geometry_msgs::Pose p;
    p.position.x = pose.first[0];
    p.position.y = pose.first[1];
    p.position.z = pose.first[2];
    p.orientation.x = pose.second.x();
    p.orientation.y = pose.second.y();
    p.orientation.z = pose.second.z();
    p.orientation.w = pose.second.w();
    pose_array.poses.push_back(p);
  }

  pubs_[PlanningVisualization::PUBLISHER::POSE_ARRAY].publish(pose_array);
}

void PlanningVisualization::drawBsplinesPhase1(vector<NonUniformBspline> &bsplines, double size) {
  vector<Eigen::Vector3d> empty;

  for (int i = 0; i < last_bspline_phase1_num_; ++i) {
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + i % 100);
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + i % 100);
  }
  last_bspline_phase1_num_ = bsplines.size();

  for (int i = 0; i < bsplines.size(); ++i) {
    drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.2), false, 2 * size,
                getColor(double(i) / bsplines.size()), i);
  }
}

void PlanningVisualization::drawBsplinesPhase2(vector<NonUniformBspline> &bsplines, double size) {
  vector<Eigen::Vector3d> empty;

  for (int i = 0; i < last_bspline_phase2_num_; ++i) {
    drawSpheres(empty, size, Eigen::Vector4d(1, 0, 0, 1), "B-Spline", i, 0);
    drawSpheres(empty, size, Eigen::Vector4d(1, 0, 0, 1), "B-Spline", i + 50, 0);
    // displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + (50
    // + i) % 100); displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1),
    // BSPLINE_CTRL_PT + (50 + i) % 100);
  }
  last_bspline_phase2_num_ = bsplines.size();

  for (int i = 0; i < bsplines.size(); ++i) {
    drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.6), false, 1.5 * size,
                getColor(double(i) / bsplines.size()), i);
  }
}

void PlanningVisualization::drawBspline(NonUniformBspline &bspline, double size,
                                        const Eigen::Vector4d &color, bool show_ctrl_pts,
                                        double size2, const Eigen::Vector4d &color2, int id1) {
  if (bspline.getControlPoint().size() == 0)
    return;

  vector<Eigen::Vector3d> traj_pts;
  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);

  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoorT(t);
    traj_pts.push_back(pt);
  }
  // displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);
  drawSpheres(traj_pts, size, color, "B-Spline", id1, 0);

  // draw the control point
  if (show_ctrl_pts) {
    Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
    vector<Eigen::Vector3d> ctp;
    for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
      Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
      ctp.push_back(pt);
    }
    // displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
    drawSpheres(ctp, size2, color2, "B-Spline", id1 + 50, 0);
  }
}

void PlanningVisualization::drawBsplineWithVelocity(NonUniformBspline &position_bspline,
                                                    NonUniformBspline &velocity_bspline) {
  if (trajectory_with_velocity_pub_.getNumSubscribers() == 0)
    return;

  if (position_bspline.getControlPoint().size() == 0 ||
      velocity_bspline.getControlPoint().size() == 0)
    return;

  pcl::PointCloud<pcl::PointXYZRGB> traj_pts;
  double tm, tmp;
  position_bspline.getTimeSpan(tm, tmp);

  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pos = position_bspline.evaluateDeBoorT(t);
    Eigen::Vector3d vel = velocity_bspline.evaluateDeBoorT(t);

    pcl::PointXYZRGB pt;
    pt.x = pos[0];
    pt.y = pos[1];
    pt.z = pos[2];

    // min vel: 0., max vel: 2.0
    double vel_norm = std::min(std::max(vel.norm() / 2.0, 0.0), 1.0);
    Eigen::Vector4d color = getColor(vel_norm);
    pt.r = color[0] * 255;
    pt.g = color[1] * 255;
    pt.b = color[2] * 255;

    traj_pts.push_back(pt);
  }

  sensor_msgs::PointCloud2 pcl_msg;
  pcl::toROSMsg(traj_pts, pcl_msg);
  pcl_msg.header.frame_id = world_frame_;
  pcl_msg.header.stamp = ros::Time::now();
  trajectory_with_velocity_pub_.publish(pcl_msg);
}

void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution,
                                     const Eigen::Vector4d &color, int id) {
  vector<Eigen::Vector3d> goal_vec = {goal};
  displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

void PlanningVisualization::drawGeometricPath(const vector<Eigen::Vector3d> &path,
                                              double resolution, const Eigen::Vector4d &color,
                                              int id) {
  displaySphereList(path, resolution, color, PATH + id % 100);
}

void PlanningVisualization::drawPolynomialTraj(PolynomialTraj poly_traj, double resolution,
                                               const Eigen::Vector4d &color, int id) {
  vector<Eigen::Vector3d> poly_pts;
  poly_traj.getSamplePoints(poly_pts);
  displaySphereList(poly_pts, resolution, color, POLY_TRAJ + id % 100);
}

void PlanningVisualization::drawFrontier(const vector<vector<Eigen::Vector3d>> &frontiers) {
  for (int i = 0; i < frontiers.size(); ++i) {
    // displayCubeList(frontiers[i], 0.1, getColor(double(i) / frontiers.size(),
    // 0.4), i, 4);
    drawCubes(frontiers[i], 1.0, getColor(double(i) / frontiers.size(), 0.8), "frontier", i,
              PUBLISHER::FRONTIER);
  }

  vector<Eigen::Vector3d> frontier;
  for (int i = frontiers.size(); i < last_frontier_num_; ++i) {
    // displayCubeList(frontier, 0.1, getColor(1), i, 4);
    drawCubes(frontier, 1.0, getColor(1), "frontier", i, PUBLISHER::FRONTIER);
  }
  last_frontier_num_ = frontiers.size();
}

void PlanningVisualization::drawYawTraj(NonUniformBspline &pos, NonUniformBspline &yaw,
                                        const double &dt) {
  double duration = pos.getTimeSum();
  vector<Eigen::Vector3d> pts1, pts2;

  for (double tc = 0.0; tc <= duration + 1e-3; tc += dt) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    pc[2] += 0.1;
    double yc = yaw.evaluateDeBoorT(tc)[0];
    Eigen::Vector3d dir(cos(yc), sin(yc), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.02, Color::Magenta(), 0, PUBLISHER::YAW_TRAJ);
}

void PlanningVisualization::drawYawWaypointsTraj(NonUniformBspline &pos, NonUniformBspline &yaw,
                                                 const double &dt,
                                                 const vector<Eigen::Vector3d> &yaw_waypts,
                                                 const vector<int> &yaw_waypts_idx) {
  CHECK_EQ(yaw_waypts.size(), yaw_waypts_idx.size()) << "yaw_waypts and yaw_waypts_idx size not "
                                                        "equal!";

  double duration = pos.getTimeSum();
  vector<Eigen::Vector3d> pts1, pts2;
  vector<Eigen::Vector3d> pts_waypts1, pts_waypts2;

  for (double tc = 0.0; tc <= duration + 1e-3; tc += dt) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    pc[2] += 0.1;
    double yc = yaw.evaluateDeBoorT(tc)[0];
    Eigen::Vector3d dir(cos(yc), sin(yc), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.02, Color::Magenta(), 0, PUBLISHER::YAW_TRAJ);

  for (int i = 0; i < yaw_waypts.size(); ++i) {
    int yaw_waypt_idx = yaw_waypts_idx[i];
    Eigen::Vector3d pc = pos.evaluateDeBoorT(yaw_waypt_idx * dt);
    pc[2] += 0.1;
    double yc = yaw_waypts[i][0];
    Eigen::Vector3d dir(cos(yc), sin(yc), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts_waypts1.push_back(pc);
    pts_waypts2.push_back(pdir);
  }
  displayLineList(pts_waypts1, pts_waypts2, 0.05, Color::Green(), 1, PUBLISHER::YAW_TRAJ);
}

void PlanningVisualization::drawYawPath(NonUniformBspline &pos, const vector<double> &yaw,
                                        const double &dt) {
  vector<Eigen::Vector3d> pts1, pts2;

  for (int i = 0; i < yaw.size(); ++i) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(i * dt);
    pc[2] += 0.3;
    Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Eigen::Vector3d pdir = pc + 0.5 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.02, Color::Magenta(), 1, PUBLISHER::YAW_TRAJ);
}

void PlanningVisualization::drawYawPath(const vector<Eigen::Vector3d> &pos,
                                        const vector<double> &yaw) {
  vector<Eigen::Vector3d> pts2;

  for (int i = 0; i < yaw.size(); ++i) {
    Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Eigen::Vector3d pdir = pos[i] + 1.0 * dir;
    pts2.push_back(pdir);
  }
  displayLineList(pos, pts2, 0.02, Color::Magenta(), 1, PUBLISHER::YAW_TRAJ);
}

Eigen::Vector4d PlanningVisualization::getColor(const double &h, double alpha) {
  double h1 = h;
  if (h1 < 0.0) {
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;
  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = (h1 - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);
  } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);
  } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);
  } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);
  } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
    lambda = (h1 - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}

void PlanningVisualization::drawFrontierPointcloud(
    const vector<vector<Eigen::Vector3d>> &frontiers) {
  if (frontier_pcl_pub_.getNumSubscribers() == 0)
    return;

  // TicToc t_pcl;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

  for (int i = 0; i < frontiers.size(); ++i) {
    auto &frontier = frontiers[i];
    for (auto pt : frontier) {
      pcl::PointXYZRGBA p;
      p.x = pt[0];
      p.y = pt[1];
      p.z = pt[2];

      Eigen::Vector4d color = getColor(double(i) / frontiers.size(), 1.0);
      p.r = color[0] * 255;
      p.g = color[1] * 255;
      p.b = color[2] * 255;
      p.a = color[3] * 255;
      pointcloud->points.push_back(p);
    }
  }

  // t_pcl.toc("draw frontier pointcloud");

  pointcloud->header.frame_id = world_frame_;
  pointcloud->width = pointcloud->points.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true; // True if no invalid points

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  frontier_pcl_pub_.publish(pointcloud_msg);
}

void PlanningVisualization::drawTinyFrontierPointcloud(
    const vector<vector<Eigen::Vector3d>> &frontiers) {
  if (tiny_frontier_pcl_pub_.getNumSubscribers() == 0)
    return;

  // TicToc t_pcl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < frontiers.size(); ++i) {
    auto &frontier = frontiers[i];
    for (auto pt : frontier) {
      pcl::PointXYZ p;
      p.x = pt[0];
      p.y = pt[1];
      p.z = pt[2];
      pointcloud->points.push_back(p);
    }
  }

  // t_pcl.toc("draw frontier pointcloud");

  pointcloud->header.frame_id = world_frame_;
  pointcloud->width = pointcloud->points.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true; // True if no invalid points

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  tiny_frontier_pcl_pub_.publish(pointcloud_msg);
}

void PlanningVisualization::drawDormantFrontierPointcloud(
    const vector<vector<Eigen::Vector3d>> &dormant_frontiers) {
  if (dormant_frontier_pcl_pub_.getNumSubscribers() == 0)
    return;

  // TicToc t_pcl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < dormant_frontiers.size(); ++i) {
    auto &frontier = dormant_frontiers[i];
    for (auto pt : frontier) {
      pcl::PointXYZ p;
      p.x = pt[0];
      p.y = pt[1];
      p.z = pt[2];
      pointcloud->points.push_back(p);
    }
  }

  // t_pcl.toc("draw frontier pointcloud");

  pointcloud->header.frame_id = world_frame_;
  pointcloud->width = pointcloud->points.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true; // True if no invalid points

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  dormant_frontier_pcl_pub_.publish(pointcloud_msg);
}

void PlanningVisualization::drawFrontierPointcloudHighResolution(
    const vector<vector<Eigen::Vector3d>> &frontiers, const double &map_resolution) {
  if (frontier_pcl_pub_.getNumSubscribers() == 0)
    return;

  // TicToc t_pcl;
  int scale = round(map_resolution / 0.1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

  for (int i = 0; i < frontiers.size(); ++i) {
    auto &frontier = frontiers[i];
    for (auto pt : frontier) {
      pcl::PointXYZRGBA p;
      p.x = pt[0];
      p.y = pt[1];
      p.z = pt[2];

      Eigen::Vector4d color = getColor(double(i) / frontiers.size(), 1.0);
      p.r = color[0] * 255;
      p.g = color[1] * 255;
      p.b = color[2] * 255;
      p.a = color[3] * 255;

      vector<pcl::PointXYZRGBA> points;
      pcl::PointXYZRGBA p1;
      p1.x = p.x - 0.05 * (scale - 1);
      p1.y = p.y - 0.05 * (scale - 1);
      p1.z = p.z - 0.05 * (scale - 1);
      for (int i = 0; i < scale; i++) {
        for (int j = 0; j < scale; j++) {
          for (int k = 0; k < scale; k++) {
            pcl::PointXYZRGBA p2 = p1;
            p2.x += 0.1 * i;
            p2.y += 0.1 * j;
            p2.z += 0.1 * k;
            p2.r = p.r;
            p2.g = p.g;
            p2.b = p.b;
            p2.a = p.a;
            points.push_back(p2);
          }
        }
      }

      pointcloud->points.insert(pointcloud->points.end(), points.begin(), points.end());
    }
  }

  // t_pcl.toc("draw frontier pointcloud");

  pointcloud->header.frame_id = world_frame_;
  pointcloud->width = pointcloud->points.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true; // True if no invalid points

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  frontier_pcl_pub_.publish(pointcloud_msg);
}

void PlanningVisualization::drawTinyFrontierPointcloudHighResolution(
    const vector<vector<Eigen::Vector3d>> &frontiers, const double &map_resolution) {
  if (tiny_frontier_pcl_pub_.getNumSubscribers() == 0)
    return;

  // TicToc t_pcl;
  int scale = round(map_resolution / 0.1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < frontiers.size(); ++i) {
    auto &frontier = frontiers[i];
    for (auto pt : frontier) {
      pcl::PointXYZ p;
      p.x = pt[0];
      p.y = pt[1];
      p.z = pt[2];

      vector<pcl::PointXYZ> points;
      pcl::PointXYZ p1;
      p1.x = p.x - 0.05 * (scale - 1);
      p1.y = p.y - 0.05 * (scale - 1);
      p1.z = p.z - 0.05 * (scale - 1);
      for (int i = 0; i < scale; i++) {
        for (int j = 0; j < scale; j++) {
          for (int k = 0; k < scale; k++) {
            pcl::PointXYZ p2 = p1;
            p2.x += 0.1 * i;
            p2.y += 0.1 * j;
            p2.z += 0.1 * k;
            points.push_back(p2);
          }
        }
      }

      pointcloud->points.insert(pointcloud->points.end(), points.begin(), points.end());
    }
  }

  // t_pcl.toc("draw frontier pointcloud");

  pointcloud->header.frame_id = world_frame_;
  pointcloud->width = pointcloud->points.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true; // True if no invalid points

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  tiny_frontier_pcl_pub_.publish(pointcloud_msg);
}

void PlanningVisualization::drawDormantFrontierPointcloudHighResolution(
    const vector<vector<Eigen::Vector3d>> &dormant_frontiers, const double &map_resolution) {
  if (dormant_frontier_pcl_pub_.getNumSubscribers() == 0)
    return;

  // TicToc t_pcl;
  int scale = round(map_resolution / 0.1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < dormant_frontiers.size(); ++i) {
    auto &frontier = dormant_frontiers[i];
    for (auto pt : frontier) {
      pcl::PointXYZ p;
      p.x = pt[0];
      p.y = pt[1];
      p.z = pt[2];

      vector<pcl::PointXYZ> points;
      pcl::PointXYZ p1;
      p1.x = p.x - 0.05 * (scale - 1);
      p1.y = p.y - 0.05 * (scale - 1);
      p1.z = p.z - 0.05 * (scale - 1);
      for (int i = 0; i < scale; i++) {
        for (int j = 0; j < scale; j++) {
          for (int k = 0; k < scale; k++) {
            pcl::PointXYZ p2 = p1;
            p2.x += 0.1 * i;
            p2.y += 0.1 * j;
            p2.z += 0.1 * k;
            points.push_back(p2);
          }
        }
      }

      pointcloud->points.insert(pointcloud->points.end(), points.begin(), points.end());
    }
  }

  // t_pcl.toc("draw frontier pointcloud");

  pointcloud->header.frame_id = world_frame_;
  pointcloud->width = pointcloud->points.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true; // True if no invalid points

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  dormant_frontier_pcl_pub_.publish(pointcloud_msg);
}

} // namespace fast_planner