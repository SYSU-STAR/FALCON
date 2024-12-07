#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <vector>

using std::shared_ptr;

class RayCaster {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<RayCaster> Ptr;
  typedef shared_ptr<const RayCaster> ConstPtr;

  RayCaster(/* args */) {}
  ~RayCaster() {}

  int signum(int x);
  double mod(double value, double modulus);
  double intbound(double s, double ds);
  void Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &min,
               const Eigen::Vector3d &max, int &output_points_cnt, Eigen::Vector3d *output);

  void Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &min,
               const Eigen::Vector3d &max, std::vector<Eigen::Vector3d> *output);

  bool input(const Eigen::Vector3d &start, const Eigen::Vector3d &end);
  bool nextId(Eigen::Vector3i &idx);
  bool nextPos(Eigen::Vector3d &pos);
  void setParams(const double &res, const Eigen::Vector3d &origin);
  bool setInput(const Eigen::Vector3d &start, const Eigen::Vector3d &end);
  bool step(Eigen::Vector3d &ray_pt);

  Eigen::Vector3d getStartPos() { return start_; }
  void getStartPos(Eigen::Vector3d &start) { start = start_; }
  Eigen::Vector3d getEndPos() { return end_; }
  void getEndPos(Eigen::Vector3d &end) { end = end_; }
  Eigen::Vector3d getStartIndex() { return Eigen::Vector3d(x_, y_, z_); }
  void getStartIndex(Eigen::Vector3i &start) { start = Eigen::Vector3i(x_, y_, z_); }
  Eigen::Vector3d getEndIndex() { return Eigen::Vector3d(endX_, endY_, endZ_); }
  void getEndIndex(Eigen::Vector3i &end) { end = Eigen::Vector3i(endX_, endY_, endZ_); }

private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int startX_;
  int startY_;
  int startZ_;
  int endX_;
  int endY_;
  int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  double dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;

  int step_num_;

  double resolution_;
  Eigen::Vector3d offset_;
  Eigen::Vector3d half_;
};

#endif // RAYCAST_H_