#include "pathfinding/path_cost_evaluator.h"

namespace fast_planner {
// Static data
double PathCostEvaluator::vm_;
double PathCostEvaluator::am_;
double PathCostEvaluator::yd_;
double PathCostEvaluator::ydd_;
Astar::Ptr PathCostEvaluator::astar_;
RayCaster::Ptr PathCostEvaluator::caster_;
MapServer::Ptr PathCostEvaluator::map_server_;

double PathCostEvaluator::computePathCost(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                 const double &y2, const Vector3d &v1, const double &yd1,
                                 const vector<Vector3d> &path, const double &len, bool verbose) {
  double pos_cost = len / vm_;

  if (len > 999.0) {
    return pos_cost; // Invalid path length，
    // 999.0 and 499.0 are magic numbers, will be removed in the future
  }

  vector<Vector3d> path_merged;
  path_merged.push_back(path[0]);
  for (int k = 1; k < (int)path.size() - 1; ++k) {
    Vector3d dir1 = (path[k] - path[k - 1]).normalized();
    Vector3d dir2 = (path[k + 1] - path[k]).normalized();
    if (dir1.dot(dir2) < 1 - 1e-4) {
      path_merged.push_back(path[k]);
    }
  }
  path_merged.push_back(path.back());

  // Consider velocity change from current to next path point
  if (v1.norm() > 1e-3) {
    Vector3d dir;
    dir = (path[1] - path[0]).normalized();
    double vc = v1.dot(dir);
    double t = pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);

    if (vc < 0) {
      t += 2 * fabs(vc) / am_;
    }
    pos_cost += t;
  }

  // Consider velocity change during path
  for (int k = 1; k < (int)path_merged.size() - 1; ++k) {
    Vector3d dir1 = (path_merged[k] - path_merged[k - 1]).normalized();
    Vector3d dir2 = (path_merged[k + 1] - path_merged[k]).normalized();
    Eigen::Vector3d vm = dir1 * vm_;
    double vc = vm.dot(dir2);
    double t = pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    if (vc < 0) {
      t += 2 * fabs(vc) / am_;
    }
    pos_cost += t;
  }

  // calculate Z diff in path
  double z_diff = 0.0;
  for (int i = 0; i < (int)path_merged.size() - 1; ++i) {
    z_diff += abs(path_merged[i + 1].z() - path_merged[i].z());
  }
  pos_cost += z_diff / 1.0; // max z velocity

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = 0.0;
  if (diff < 0.5 * yd_ * yd_ / ydd_)
    yaw_cost = sqrt(2 * diff / ydd_);
  else
    yaw_cost = yd_ / ydd_ + (diff - 0.5 * yd_ * yd_ / ydd_) / yd_;

  return max(pos_cost, yaw_cost);
}

double PathCostEvaluator::searchPath(const Vector3d &p1, const Vector3d &p2, vector<Vector3d> &path) {
  // Try connect two points with straight line
  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::OCCUPIED ||
        map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::UNKNOWN ||
        !map_server_->getTSDF()->isInBox(idx)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = {p1, p2};
    return (p1 - p2).norm();
  }
  // Search a path using decreasing resolution
  // vector<double> res = {0.2};
  // for (int k = 0; k < res.size(); ++k) {
  astar_->reset();
  // astar_->setResolution(res[k]);
  if (astar_->search(p1, p2) == Astar::REACH_END) {
    path = astar_->getPath();
    return astar_->pathLength(path);
  }
  // }
  // Use Astar early termination cost as an estimate
  path = {p1, p2};
  return 1000.0 + (p1 - p2).norm();
}

double PathCostEvaluator::searchPathBBox(const Vector3d &p1, const Vector3d &p2, const Vector3d &bbox_min,
                                const Vector3d &bbox_max, vector<Vector3d> &path) {
  // Try connect two points with straight line
  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::OCCUPIED ||
        map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::UNKNOWN ||
        !map_server_->getTSDF()->isInBox(idx)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = {p1, p2};
    return (p1 - p2).norm();
  }
  // Search a path using decreasing resolution
  // vector<double> res = {0.2};
  // for (int k = 0; k < res.size(); ++k) {
  astar_->reset();
  // astar_->setResolution(res[k]);
  if (astar_->searchBBox(p1, p2, bbox_min, bbox_max) == Astar::REACH_END) {
    path = astar_->getPath();
    return astar_->pathLength(path);
  }
  // }
  path = {p1, p2};
  return 1000.0 + (p1 - p2).norm();
}

double PathCostEvaluator::searchPathUnknown(const Vector3d &p1, const Vector3d &p2, vector<Vector3d> &path) {
  // Try connect two points with straight line
  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::OCCUPIED ||
        !map_server_->getTSDF()->isInBox(idx)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = {p1, p2};
    return (p1 - p2).norm();
  }

  // Try astar search
  astar_->reset();
  if (astar_->searchUnknown(p1, p2) == Astar::REACH_END) {
    path = astar_->getPath();
    astar_->shortenPath(path);
    return astar_->pathLength(path);
  }

  // Search failed, path is straight line with high cost
  path = {p1, p2};
  return 1000.0 + (p1 - p2).norm();
}

double PathCostEvaluator::searchPathUnknownBBox(const Vector3d &p1, const Vector3d &p2,
                                       const Vector3d &bbox_min, const Vector3d &bbox_max,
                                       vector<Vector3d> &path) {
  // Try connect two points with straight line
  auto isInBox = [&](const Vector3d &p) {
    return p.x() > bbox_min.x() && p.x() < bbox_max.x() && p.y() > bbox_min.y() &&
           p.y() < bbox_max.y() && p.z() > bbox_min.z() && p.z() < bbox_max.z();
  };

  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    Vector3d p = map_server_->getOccupancyGrid()->indexToPosition(idx);
    if (map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::OCCUPIED ||
        !isInBox(p)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = {p1, p2};
    return (p1 - p2).norm();
  }
  // Search a path using decreasing resolution
  // vector<double> res = {0.2};
  // for (int k = 0; k < res.size(); ++k) {
  astar_->reset();
  // astar_->setResolution(res[k]);
  if (astar_->searchUnknownBBox(p1, p2, bbox_min, bbox_max) == Astar::REACH_END) {
    path = astar_->getPath();
    return astar_->pathLength(path);
  }
  // }
  path = {p1, p2};
  return 1000.0 + (p1 - p2).norm();
  // return (p1 - p2).norm() * 10.0;
}

double PathCostEvaluator::searchPathUnknownOnlyBBox(const Vector3d &p1, const Vector3d &p2,
                                           const Vector3d &bbox_min, const Vector3d &bbox_max,
                                           vector<Vector3d> &path) {
  // Try connect two points with straight line
  auto isInBox = [&](const Vector3d &p) {
    return p.x() > bbox_min.x() && p.x() < bbox_max.x() && p.y() > bbox_min.y() &&
           p.y() < bbox_max.y() && p.z() > bbox_min.z() && p.z() < bbox_max.z();
  };

  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    Vector3d p = map_server_->getOccupancyGrid()->indexToPosition(idx);
    if (map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::OCCUPIED ||
        map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::FREE ||
        !isInBox(p)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = {p1, p2};
    return (p1 - p2).norm();
  }
  // Search a path using decreasing resolution
  // vector<double> res = {0.2};
  // for (int k = 0; k < res.size(); ++k) {
  astar_->reset();
  // astar_->setResolution(res[k]);
  if (astar_->searchUnknownOnlyBBox(p1, p2, bbox_min, bbox_max) == Astar::REACH_END) {
    path = astar_->getPath();
    return astar_->pathLength(path);
  }
  // }
  path = {p1, p2};
  return 1000.0 + (p1 - p2).norm();
  // return (p1 - p2).norm() * 10.0;
}

// Input:
// p1, p2: start and end position
// y1, y2: start and end yaw
// v1: start velocity
// yd1: start yaw rate
// Output:
// path: path from p1 to p2
double PathCostEvaluator::computeCost(const Vector3d &p1, const Vector3d &p2, const double &y1,
                             const double &y2, const Vector3d &v1, const double &yd1,
                             vector<Vector3d> &path, bool verbose) {
  // Cost of position change
  double pos_cost = PathCostEvaluator::searchPath(p1, p2, path);

  return computePathCost(p1, p2, y1, y2, v1, yd1, path, pos_cost);

  if (verbose) {
    std::cout << "path length: " << pos_cost << std::endl;
    std::cout << "path: " << std::endl;
    for (int k = 0; k < (int)path.size(); ++k)
      std::cout << path[k].transpose() << " | ";
  }

  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir;
    if (path.size() < 2)
      dir = (p2 - p1).normalized();
    else
      dir = (path[1] - path[0]).normalized();
    double vc = v1.dot(dir);
    double t = pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    if (vc < 0)
      t += 2 * fabs(vc) / am_;
    pos_cost += t;

    if (verbose) {
      std::cout << "velocity change: " << t << std::endl;
    }
  }

  // // Cost of yaw change
  // double diff = fabs(y2 - y1);
  // diff = min(diff, 2 * M_PI - diff);
  // double yaw_cost = diff / yd_;
  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = 0.0;
  if (diff < 0.5 * yd_ * yd_ / ydd_)
    yaw_cost = sqrt(2 * diff / ydd_);
  else
    yaw_cost = yd_ / ydd_ + (diff - 0.5 * yd_ * yd_ / ydd_) / yd_;

  if (verbose) {
    std::cout << "pos_cost: " << pos_cost << ", "
              << "yaw_cost: " << yaw_cost << std::endl;
  }

  return max(pos_cost, yaw_cost);

  // // Consider yaw rate change
  // if (fabs(yd1) > 1e-3)
  // {
  //   double diff1 = y2 - y1;
  //   while (diff1 < -M_PI)
  //     diff1 += 2 * M_PI;
  //   while (diff1 > M_PI)
  //     diff1 -= 2 * M_PI;
  //   double diff2 = diff1 > 0 ? diff1 - 2 * M_PI : 2 * M_PI + diff1;
  // }
  // else
  // {
  // }
}

double PathCostEvaluator::computeCostBBox(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                 const double &y2, const Vector3d &v1, const double &yd1,
                                 const Vector3d &bbox_min, const Vector3d &bbox_max,
                                 vector<Vector3d> &path, bool verbose) {
  // Cost of position change
  double pos_cost = PathCostEvaluator::searchPathBBox(p1, p2, bbox_min, bbox_max, path) / vm_;

  if (verbose) {
    std::cout << "path length: " << pos_cost << std::endl;
    std::cout << "path: " << std::endl;
    for (int k = 0; k < (int)path.size(); ++k)
      std::cout << path[k].transpose() << " | ";
  }

  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir;
    if (path.size() < 2)
      dir = (p2 - p1).normalized();
    else
      dir = (path[1] - path[0]).normalized();
    double vc = v1.dot(dir);
    double t = pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    if (vc < 0)
      t += 2 * fabs(vc) / am_;
    pos_cost += t;

    if (verbose) {
      std::cout << "velocity change: " << t << std::endl;
    }
  }

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = 0.0;
  if (diff < 0.5 * yd_ * yd_ / ydd_)
    yaw_cost = sqrt(2 * diff / ydd_);
  else
    yaw_cost = yd_ / ydd_ + (diff - 0.5 * yd_ * yd_ / ydd_) / yd_;

  if (verbose) {
    std::cout << "pos_cost: " << pos_cost << ", "
              << "yaw_cost: " << yaw_cost << std::endl;
  }

  return max(pos_cost, yaw_cost);
}

// Consider unknown space as free space
// TODO: penalize unknown space
double PathCostEvaluator::computeCostUnknown(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                    const double &y2, const Vector3d &v1, const double &yd1,
                                    vector<Vector3d> &path) {
  double len = PathCostEvaluator::searchPathUnknown(p1, p2, path);
  return computePathCost(p1, p2, y1, y2, v1, yd1, path, len);
}

double PathCostEvaluator::computeCostUnknownBBox(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                        const double &y2, const Vector3d &v1, const double &yd1,
                                        const Vector3d &bbox_min, const Vector3d &bbox_max,
                                        vector<Vector3d> &path) {
  // Cost of position change
  double pos_cost = PathCostEvaluator::searchPathUnknownBBox(p1, p2, bbox_min, bbox_max, path) / vm_;

  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir;
    if (path.size() < 2)
      dir = (p2 - p1).normalized();
    else
      dir = (path[1] - path[0]).normalized();
    double vc = v1.dot(dir);
    double t = pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    if (vc < 0)
      t += 2 * fabs(vc) / am_;
    pos_cost += t;
  }

  // // Cost of yaw change
  // double diff = fabs(y2 - y1);
  // diff = min(diff, 2 * M_PI - diff);
  // double yaw_cost = diff / yd_;

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = 0.0;
  if (diff < 0.5 * yd_ * yd_ / ydd_)
    yaw_cost = sqrt(2 * diff / ydd_);
  else
    yaw_cost = yd_ / ydd_ + (diff - 0.5 * yd_ * yd_ / ydd_) / yd_;

  return max(pos_cost, yaw_cost);
}

double PathCostEvaluator::computeCostUnknownOnlyBBox(const Vector3d &p1, const Vector3d &p2,
                                            const double &y1, const double &y2, const Vector3d &v1,
                                            const double &yd1, const Vector3d &bbox_min,
                                            const Vector3d &bbox_max, vector<Vector3d> &path) {
  // Cost of position change
  double pos_cost = PathCostEvaluator::searchPathUnknownOnlyBBox(p1, p2, bbox_min, bbox_max, path) / vm_;

  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir;
    if (path.size() < 2)
      dir = (p2 - p1).normalized();
    else
      dir = (path[1] - path[0]).normalized();
    double vc = v1.dot(dir);
    double t = pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    if (vc < 0)
      t += 2 * fabs(vc) / am_;
    pos_cost += t;
  }

  // // Cost of yaw change
  // double diff = fabs(y2 - y1);
  // diff = min(diff, 2 * M_PI - diff);
  // double yaw_cost = diff / yd_;

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = 0.0;
  if (diff < 0.5 * yd_ * yd_ / ydd_)
    yaw_cost = sqrt(2 * diff / ydd_);
  else
    yaw_cost = yd_ / ydd_ + (diff - 0.5 * yd_ * yd_ / ydd_) / yd_;

  return max(pos_cost, yaw_cost);
}
} // namespace fast_planner