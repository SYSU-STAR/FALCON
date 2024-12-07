#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <algorithm>
#include <iostream>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "bspline/non_uniform_bspline.h"
#include "exploration_types.h"
#include "polynomial/polynomial_traj.h"

using std::vector;

namespace fast_planner {
class PlanningVisualization {
public:
  typedef shared_ptr<PlanningVisualization> Ptr;
  typedef shared_ptr<const PlanningVisualization> ConstPtr;

  PlanningVisualization(/* args */) {}
  ~PlanningVisualization() {}
  PlanningVisualization(ros::NodeHandle &nh);

  // new interface
  void fillBasicInfo(visualization_msgs::Marker &mk, const Eigen::Vector3d &scale,
                     const Eigen::Vector4d &color, const string &ns, const int &id,
                     const int &shape);
  void fillGeometryInfo(visualization_msgs::Marker &mk, const vector<Eigen::Vector3d> &list);
  void fillGeometryInfo(visualization_msgs::Marker &mk, const vector<Eigen::Vector3d> &list1,
                        const vector<Eigen::Vector3d> &list2);

  void drawSpheres(const vector<Eigen::Vector3d> &list, const double &scale,
                   const Eigen::Vector4d &color, const string &ns, const int &id,
                   const int &pub_id);
  void drawCubes(const vector<Eigen::Vector3d> &list, const double &scale,
                 const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);
  void drawLines(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2,
                 const double &scale, const Eigen::Vector4d &color, const string &ns, const int &id,
                 const int &pub_id);
  void drawLines(const vector<Eigen::Vector3d> &list, const double &scale,
                 const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);
  void drawLinesList(const vector<vector<Eigen::Vector3d>> &lines_list, const double &scale,
                     const string &ns, const int &pub_id);
  void drawMidArrowLines(const vector<Eigen::Vector3d> &list, const double &scale,
                         const Eigen::Vector4d &color, const string &ns, const int &id,
                         const int &pub_id);
  void drawBox(const Eigen::Vector3d &center, const Eigen::Vector3d &scale,
               const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);
  void drawText(const Eigen::Vector3d &pos, const string &text, const double &scale,
                const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);
  void removeText(const string &ns, const int &id, const int &pub_id);
  void drawPose(const Position &position, const Eigen::Quaterniond &orientation, const string &ns,
                const int &id);
  void drawPoses(const vector<pair<Position, Eigen::Quaterniond>> &poses, const string &ns);

  // Deprecated
  // draw basic shapes
  void displaySphereList(const vector<Eigen::Vector3d> &list, double resolution,
                         const Eigen::Vector4d &color, int id, int pub_id = 0);
  void displayCubeList(const vector<Eigen::Vector3d> &list, double resolution,
                       const Eigen::Vector4d &color, int id, int pub_id = 0);
  void displayLineList(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2,
                       double line_width, const Eigen::Vector4d &color, int id, int pub_id = 0);
  // draw a piece-wise straight line path
  void drawGeometricPath(const vector<Eigen::Vector3d> &path, double resolution,
                         const Eigen::Vector4d &color, int id = 0);
  // draw a polynomial trajectory
  void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d &color,
                          int id = 0);
  // draw a bspline trajectory
  void drawBspline(NonUniformBspline &bspline, double size, const Eigen::Vector4d &color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d &color2 = Eigen::Vector4d(1, 1, 0, 1), int id = 0);
  void drawBsplineWithVelocity(NonUniformBspline &position_bspline,
                               NonUniformBspline &velocity_bspline);
  // draw a set of bspline trajectories generated in different phases
  void drawBsplinesPhase1(vector<NonUniformBspline> &bsplines, double size);
  void drawBsplinesPhase2(vector<NonUniformBspline> &bsplines, double size);

  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d &color, int id = 0);

  Eigen::Vector4d getColor(const double &h, double alpha = 1.0);

  void drawFrontier(const vector<vector<Eigen::Vector3d>> &frontiers);
  void drawYawTraj(NonUniformBspline &pos, NonUniformBspline &yaw, const double &dt);
  void drawYawWaypointsTraj(NonUniformBspline &pos, NonUniformBspline &yaw, const double &dt,
                            const vector<Eigen::Vector3d> &yaw_waypts,
                            const vector<int> &yaw_waypts_idx);
  void drawYawPath(NonUniformBspline &pos, const vector<double> &yaw, const double &dt);
  void drawYawPath(const vector<Eigen::Vector3d> &pos, const vector<double> &yaw);

  void drawFrontierPointcloud(const vector<vector<Eigen::Vector3d>> &frontiers);
  void drawDormantFrontierPointcloud(const vector<vector<Eigen::Vector3d>> &dormant_frontiers);
  void drawTinyFrontierPointcloud(const vector<vector<Eigen::Vector3d>> &frontiers);
  void drawFrontierPointcloudHighResolution(const vector<vector<Eigen::Vector3d>> &frontiers,
                                            const double &map_resolution);
  void drawDormantFrontierPointcloudHighResolution(
      const vector<vector<Eigen::Vector3d>> &dormant_frontiers, const double &map_resolution);
  void drawTinyFrontierPointcloudHighResolution(const vector<vector<Eigen::Vector3d>> &frontiers,
                                                const double &map_resolution);

  enum PUBLISHER {
    TRAJECTORY = 0,
    TOPOLOGICAL_PATH = 1,
    PREDICTION = 2,
    VISIBILITY = 3,
    FRONTIER = 4,
    YAW_TRAJ = 5,
    VIEWPOINT = 6,
    HGRID = 7,
    DEBUG = 8,
    POSE = 9,
    POSE_ARRAY = 10
  };

  struct Color {
    // r, g, b, a: range from 0.0 to 1.0
    double r_;
    double g_;
    double b_;
    double a_;

    Color() : r_(0), g_(0), b_(0), a_(1) {}
    Color(double r, double g, double b) : Color(r, g, b, 1.) {}
    Color(double r, double g, double b, double a) : r_(r), g_(g), b_(b), a_(a) {}
    Color(int r, int g, int b) {
      r_ = static_cast<double>(r) / 255.;
      g_ = static_cast<double>(g) / 255.;
      b_ = static_cast<double>(b) / 255.;
      a_ = 1.;
    }
    Color(int r, int g, int b, int a) {
      r_ = static_cast<double>(r) / 255.;
      g_ = static_cast<double>(g) / 255.;
      b_ = static_cast<double>(b) / 255.;
      a_ = static_cast<double>(a) / 255.;
    }

    static Eigen::Vector4d toEigen(const Color &color) {
      return Eigen::Vector4d(color.r_, color.g_, color.b_, color.a_);
    }

    static Eigen::Vector4i toEigenInt(const Color &color) {
      return Eigen::Vector4i(color.r_ * 255, color.g_ * 255, color.b_ * 255, color.a_ * 255);
    }

    static const Eigen::Vector4d White() { return toEigen(Color(255, 255, 255)); }
    static const Eigen::Vector4d Black() { return toEigen(Color(0, 0, 0)); }
    static const Eigen::Vector4d Gray() { return toEigen(Color(127, 127, 127)); }
    static const Eigen::Vector4d Red() { return toEigen(Color(255, 0, 0)); }
    static const Eigen::Vector4d DeepRed() { return toEigen(Color(127, 0, 0)); }
    static const Eigen::Vector4d Green() { return toEigen(Color(0, 255, 0)); }
    static const Eigen::Vector4d DeepGreen() { return toEigen(Color(0, 127, 0)); }
    static const Eigen::Vector4d SpringGreen() { return toEigen(Color(0, 255, 127)); }
    static const Eigen::Vector4d LightBlue() { return toEigen(Color(173, 216, 230)); }
    static const Eigen::Vector4d Blue() { return toEigen(Color(0, 0, 255)); }
    static const Eigen::Vector4d Blue2() { return toEigen(Color(0, 175, 240)); }
    static const Eigen::Vector4d DeepBlue() { return toEigen(Color(0, 0, 127)); }
    static const Eigen::Vector4d Yellow() { return toEigen(Color(255, 255, 0)); }
    static const Eigen::Vector4d Orange() { return toEigen(Color(255, 127, 0)); }
    static const Eigen::Vector4d Purple() { return toEigen(Color(127, 0, 255)); }
    static const Eigen::Vector4d Teal() { return toEigen(Color(0, 255, 255)); }
    static const Eigen::Vector4d TealTransparent() { return toEigen(Color(0, 255, 255, 200)); }
    static const Eigen::Vector4d Pink() { return toEigen(Color(255, 0, 127)); }
    static const Eigen::Vector4d Magenta() { return toEigen(Color(255, 0, 255)); }
    static const Eigen::Vector4d Cyan() { return toEigen(Color(0, 255, 255)); }
  };

private:
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    POLY_TRAJ = 500
  };

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;
  ros::Publisher traj_pub_;       // 0
  ros::Publisher topo_pub_;       // 1
  ros::Publisher predict_pub_;    // 2
  ros::Publisher visib_pub_;      // 3, visibility constraints
  ros::Publisher frontier_pub_;   // 4, frontier searching
  ros::Publisher yaw_pub_;        // 5, yaw trajectory
  ros::Publisher viewpoint_pub_;  // 6, viewpoint planning
  ros::Publisher hgrid_pub_;      // 7, hierarchical grid
  ros::Publisher debug_pub_;      // 8, marker debug
  ros::Publisher pose_pub_;       // 9, pose
  ros::Publisher pose_array_pub_; // 10, pose array
  vector<ros::Publisher> pubs_;

  ros::Publisher frontier_pcl_pub_, dormant_frontier_pcl_pub_, tiny_frontier_pcl_pub_,
      trajectory_with_velocity_pub_;

  int last_topo_path1_num_;
  int last_topo_path2_num_;
  int last_bspline_phase1_num_;
  int last_bspline_phase2_num_;
  int last_frontier_num_;

  string world_frame_;
};
} // namespace fast_planner
#endif