#include <Eigen/Eigenvalues>
#include <pcl/filters/voxel_grid.h>

#include "exploration_preprocessing/frontier_finder.h"
#include "pathfinding/path_cost_evaluator.h"
#include "perception_utils/perception_utils.h"

namespace fast_planner {
FrontierFinder::FrontierFinder(ros::NodeHandle &nh, const shared_ptr<MapServer> &map_server) {
  this->map_server_ = map_server;
  int voxel_num = map_server_->getVoxelNum();
  frontier_flag_ = vector<bool>(voxel_num, false);

  nh.param("/frontier_finder/cluster_min", cluster_min_, -1);
  nh.param("/frontier_finder/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("/frontier_finder/cluster_size_z", cluster_size_z_, -1.0);
  nh.param("/frontier_finder/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("/frontier_finder/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("/frontier_finder/min_candidate_occupied_clearance", min_candidate_occupied_clearance_,
           -1.0);
  nh.param("/frontier_finder/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("/frontier_finder/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("/frontier_finder/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("/frontier_finder/candidate_rnum", candidate_rnum_, -1);
  nh.param("/frontier_finder/down_sample", down_sample_, -1);
  nh.param("/frontier_finder/min_visib_num", min_visib_num_, -1);
  nh.param("/frontier_finder/min_view_finish_fraction", min_view_finish_fraction_, -1.0);
  nh.param("/frontier_finder/ground_height", ground_height_, 0.0);
  nh.param("/frontier_finder/ground_offset", ground_offset_, 0.0);
  nh.param("/frontier_finder/viewpoint_z_score_cutoff", viewpoint_z_score_cutoff_, 0.0);
  nh.param("/frontier_finder/cell_height_clearance", cell_height_clearance_, 0.1);

  raycaster_.reset(new RayCaster);
  resolution_ = map_server_->getResolution();
  Eigen::Vector3d origin, size;
  map_server_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);

  // For low resolution map, decrease the cluster_min_ and min_visib_num_
  if (resolution_ > 0.15) {
    cluster_min_ /= 2;
    min_visib_num_ *= 0.8;
    ROS_INFO("[FrontierFinder] Low resolution map detected, decrease cluster_min to %d, decrease "
             "min_visib_num to %d",
             cluster_min_, min_visib_num_);
  }

  percep_utils_.reset(new PerceptionUtils(nh));
}

FrontierFinder::~FrontierFinder() {}

void FrontierFinder::searchFrontiers() {
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();

  // Bounding box of updated region
  Vector3d update_min, update_max;
  map_server_->getTSDF()->getUpdatedBox(update_min, update_max, true);
  update_bbox_min_ = update_min;
  update_bbox_max_ = update_max;
  // ROS_INFO("[FrontierFinder] Updated box: (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f)",
  // update_min(0),
  //          update_min(1), update_min(2), update_max(0), update_max(1), update_max(2));

  // Removed changed frontiers flags within update bbox
  auto resetFlag = [&](list<Frontier>::iterator &iter, list<Frontier> &frontiers) {
    Eigen::Vector3i idx;
    for (auto cell : iter->cells_) {
      map_server_->getTSDF()->positionToIndex(cell, idx);
      frontier_flag_[toadr(idx)] = false;
    }
    iter = frontiers.erase(iter);
  };

  // std::cout << "Before remove: " << frontiers_.size() << std::endl;

  removed_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter)) {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx;
      ++iter;
    }
  }
  // std::cout << "After remove: " << frontiers_.size() << std::endl;
  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }
  for (auto iter = tiny_frontiers_.begin(); iter != tiny_frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter))
      resetFlag(iter, tiny_frontiers_);
    else
      ++iter;
  }

  // Search new frontier within box slightly inflated from updated box
  // Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
  // Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
  Vector3d search_min = update_min;
  Vector3d search_max = update_max;
  Vector3d box_min, box_max;
  map_server_->getBox(box_min, box_max);
  for (int k = 0; k < 3; ++k) {
    search_min[k] = max(search_min[k], box_min[k]);
    search_max[k] = min(search_max[k], box_max[k]);
  }
  Eigen::Vector3i min_id, max_id;
  map_server_->getTSDF()->positionToIndex(search_min, min_id);
  map_server_->getTSDF()->positionToIndex(search_max, max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        // Scanning the updated region to find seeds of frontiers
        Eigen::Vector3i cur(x, y, z);
        if (frontier_flag_[toadr(cur)] == false && knownfree(cur) && isNeighborUnknown(cur)) {
          // Expand from the seed cell to find a complete frontier cluster
          expandFrontier(cur);
        }
      }
  splitLargeFrontiers(tmp_frontiers_);

  // ROS_INFO("[FrontierFinder] Frontier search time: %lf", (ros::Time::now() - t1).toSec());
}

void FrontierFinder::expandFrontier(
    const Eigen::Vector3i &first /* , const int& depth, const int& parent_id */) {
  // std::cout << "depth: " << depth << std::endl;
  auto t1 = ros::Time::now();

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;
  Vector2d cluster_mean;
  double z_mean;

  // map bbox
  Vector3d box_min, box_max;
  map_server_->getBox(box_min, box_max);

  Vector3i bbox_min_idx, bbox_max_idx;
  map_server_->getBoxIndex(bbox_min_idx, bbox_max_idx);

  map_server_->getTSDF()->indexToPosition(first, pos);

  auto isNearCellHeight = [&](const Eigen::Vector3d &pos) {
    for (auto height : cell_heights_) {
      // if (abs(pos[2] - height) < cell_height_clearance_) {
      if (height - pos[2] < cell_height_clearance_ && height - pos[2] > 0) {
        return true;
      }
    }

    if (abs(pos[2] - cell_heights_.back()) < cell_height_clearance_) {
      return true;
    }

    return false;
  };

  if (isNearCellHeight(pos)) {
    return;
  }

  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = true;

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and frontier cell not
      // clustered
      int adr = toadr(nbr);
      if (frontier_flag_[adr] == true || !map_server_->isInBox(nbr) ||
          !(knownfree(nbr) && isNeighborUnknown(nbr))) {
        continue;
      }

      map_server_->getTSDF()->indexToPosition(nbr, pos);

      // Remove noise close to ground
      if (pos[2] < ground_height_ + ground_offset_) {
        continue;
      }

      // Partition frontiers at cell heights
      if (isNearCellHeight(pos)) {
        continue;
      }

      expanded.push_back(pos);
      cluster_mean += Vector2d(pos[0], pos[1]);
      z_mean += pos[2];
      cell_queue.push(nbr);
      frontier_flag_[adr] = true;
    }
  }
  cluster_mean /= double(expanded.size());
  z_mean /= double(expanded.size());

  // if the expanded cluster is near bbox corner, the cluster min will be set small to ensure a full
  // coverage
  vector<Eigen::Vector2d> bbox_corners = {{box_min[0], box_min[1]},
                                          {box_min[0], box_max[1]},
                                          {box_max[0], box_min[1]},
                                          {box_max[0], box_max[1]}};
  bool isNearBBoxCorner = false;
  for (auto corner : bbox_corners) {
    if ((corner - cluster_mean).norm() < 2.0) {
      isNearBBoxCorner = true;
      break;
    }
  }

  int cluster_min = cluster_min_;

  // Disable above function
  // if (isNearBBoxCorner) {
  //   cluster_min = 5;
  // }

  if (expanded.size() > cluster_min) {
    // // Check z pos distribution
    // double z_abs_err = 0.0;
    // bool flatFrontier = true;
    // int cnt = 1;
    // for (auto cell : expanded) {
    //   z_abs_err += fabs(cell[2] - z_mean);
    //   if ((z_abs_err / cnt) > 1.0) {
    //     flatFrontier = false;
    //     break;
    //   }
    //   cnt++;
    // }
    // if (flatFrontier) {
    //   // Remove flat clusters
    //   for (auto cell : expanded) {
    //     Eigen::Vector3i idx;
    //     map_server_->getTSDF()->positionToIndex(cell, idx);
    //     frontier_flag_[toadr(idx)] = false;
    //   }
    //   return;
    // }

    // Compute detailed info
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  } else {
    // Tiny frontier hole filling
    Frontier tiny_frontier;
    tiny_frontier.cells_ = expanded;
    computeFrontierInfo(tiny_frontier);

    auto hasOccupiedNearby = [&](const Eigen::Vector3d &pos) {
      Eigen::Vector3i idx;
      map_server_->getTSDF()->positionToIndex(pos, idx);
      auto nbrs = allNeighbors(idx);
      for (auto nbr : nbrs) {
        if (map_server_->getOccupancy(nbr) == voxel_mapping::OccupancyType::OCCUPIED) {
          return true;
        }
      }
      return false;
    };

    auto findNearbyUnknownVoxels = [&](const vector<Eigen::Vector3d> &frontier_voxels,
                                       vector<Eigen::Vector3d> &unknown_voxels) {
      for (auto voxel : frontier_voxels) {
        Eigen::Vector3i idx;
        map_server_->getTSDF()->positionToIndex(voxel, idx);
        auto nbrs = allNeighbors(idx);
        for (auto nbr : nbrs) {
          if (map_server_->getOccupancy(nbr) == voxel_mapping::OccupancyType::UNKNOWN) {
            Eigen::Vector3d pos;
            map_server_->getTSDF()->indexToPosition(nbr, pos);
            unknown_voxels.push_back(pos);
          }
        }
      }
    };

    auto findNearbyFreeVoxels = [&](const vector<Eigen::Vector3d> &frontier_voxels,
                                    vector<Eigen::Vector3d> &free_voxels) {
      for (auto voxel : frontier_voxels) {
        Eigen::Vector3i idx;
        map_server_->getTSDF()->positionToIndex(voxel, idx);
        auto nbrs = allNeighbors(idx);
        for (auto nbr : nbrs) {
          if (map_server_->getOccupancy(nbr) == voxel_mapping::OccupancyType::FREE) {
            Eigen::Vector3d pos;
            map_server_->getTSDF()->indexToPosition(nbr, pos);
            free_voxels.push_back(pos);
          }
        }
      }
    };

    vector<Eigen::Vector3d> unknown_voxels;
    findNearbyUnknownVoxels(tiny_frontier.cells_, unknown_voxels);
    expanded.insert(expanded.end(), unknown_voxels.begin(), unknown_voxels.end());

    vector<Eigen::Vector3d> free_voxels;
    findNearbyFreeVoxels(tiny_frontier.cells_, free_voxels);
    expanded.insert(expanded.end(), free_voxels.begin(), free_voxels.end());

    std::set<int> expanded_addrs;
    for (auto cell : expanded) {
      Eigen::Vector3i idx;
      map_server_->getTSDF()->positionToIndex(cell, idx);
      expanded_addrs.insert(toadr(idx));
    }

    auto isOccupiedBoundedVoxel = [&](const Eigen::Vector3d &pos) {
      Eigen::Vector3i idx;
      map_server_->getTSDF()->positionToIndex(pos, idx);

      bool isXPositiveOccupiedBounded = false;
      bool isXNegativeOccupiedBounded = false;
      bool isYPositiveOccupiedBounded = false;
      bool isYNegativeOccupiedBounded = false;
      bool isZPositiveOccupiedBounded = false;
      bool isZNegativeOccupiedBounded = false;

      // Find through + and - x-axis, if there is occupied voxel next to the frontier cell
      Eigen::Vector3i idx_x = idx;
      while (idx_x[0] < bbox_max_idx[0] &&
             expanded_addrs.find(toadr(idx_x)) != expanded_addrs.end()) {
        idx_x[0]++;
        if (map_server_->getOccupancy(idx_x) == voxel_mapping::OccupancyType::OCCUPIED) {
          isXPositiveOccupiedBounded = true;
          break;
        }
      }

      idx_x = idx;
      while (idx_x[0] > bbox_min_idx[0] &&
             expanded_addrs.find(toadr(idx_x)) != expanded_addrs.end()) {
        idx_x[0]--;
        if (map_server_->getOccupancy(idx_x) == voxel_mapping::OccupancyType::OCCUPIED) {
          isXNegativeOccupiedBounded = true;
          break;
        }
      }

      if (isXPositiveOccupiedBounded && isXNegativeOccupiedBounded) {
        return true;
      }

      // Find through + and - y-axis, if there is occupied voxel next to the frontier cell
      Eigen::Vector3i idx_y = idx;
      while (idx_y[1] < bbox_max_idx[1] &&
             expanded_addrs.find(toadr(idx_y)) != expanded_addrs.end()) {
        idx_y[1]++;
        if (map_server_->getOccupancy(idx_y) == voxel_mapping::OccupancyType::OCCUPIED) {
          isYPositiveOccupiedBounded = true;
          break;
        }
      }

      idx_y = idx;
      while (idx_y[1] > bbox_min_idx[1] &&
             expanded_addrs.find(toadr(idx_y)) != expanded_addrs.end()) {
        idx_y[1]--;
        if (map_server_->getOccupancy(idx_y) == voxel_mapping::OccupancyType::OCCUPIED) {
          isYNegativeOccupiedBounded = true;
          break;
        }
      }

      if (isYPositiveOccupiedBounded && isYNegativeOccupiedBounded) {
        return true;
      }

      // Find through + and - z-axis, if there is occupied voxel next to the frontier cell
      Eigen::Vector3i idx_z = idx;
      while (idx_z[2] < bbox_max_idx[2] &&
             expanded_addrs.find(toadr(idx_z)) != expanded_addrs.end()) {
        idx_z[2]++;
        if (map_server_->getOccupancy(idx_z) == voxel_mapping::OccupancyType::OCCUPIED) {
          isZPositiveOccupiedBounded = true;
          break;
        }
      }

      idx_z = idx;
      while (idx_z[2] > bbox_min_idx[2] &&
             expanded_addrs.find(toadr(idx_z)) != expanded_addrs.end()) {
        idx_z[2]--;
        if (map_server_->getOccupancy(idx_z) == voxel_mapping::OccupancyType::OCCUPIED) {
          isZNegativeOccupiedBounded = true;
          break;
        }
      }

      if (isZPositiveOccupiedBounded && isZNegativeOccupiedBounded) {
        return true;
      }

      return false;
    };

    // if voxels in expanded is bounded by occupied voxels in x-axis or y-axis or z-axis, then it is
    // a tiny frontier
    for (auto voxel : expanded) {
      if (!map_server_->isInBox(voxel)) {
        continue;
      }
      if (isOccupiedBoundedVoxel(voxel))
        tiny_frontier.expended_cells_.push_back(voxel);
    }

    if (tiny_frontier.cells_.size() > 0) {
      tiny_frontiers_.push_back(tiny_frontier);
    }

    // Remove small clusters
    // for (auto cell : expanded) {
    //   Eigen::Vector3i idx;
    //   map_server_->getTSDF()->positionToIndex(cell, idx);
    //   frontier_flag_[toadr(idx)] = false;
    // }
  }
}

void FrontierFinder::splitLargeFrontiers(list<Frontier> &frontiers) {
  list<Frontier> splits, tmps;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(*it, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(*it);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitHorizontally(Frontier &frontier, list<Frontier> &splits) {
  // Split a frontier into small piece if it is too large
  auto mean = frontier.average_.head<2>();
  bool need_split = false;
  for (auto cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }

  // Compute principal component (PC)
  // Covariance matrix of cells
  Eigen::Matrix2d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);
  // std::cout << "max idx: " << max_idx << std::endl;
  // std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() <<
  // std::endl;

  if (!need_split) {
    frontier.first_pc_ = first_pc;
    return false;
  }

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  list<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

bool FrontierFinder::haveOverlap(const Vector3d &min1, const Vector3d &max1, const Vector3d &min2,
                                 const Vector3d &max2) {
  // Check if two box have overlap part
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3)
      return false;
  }
  return true;
}

bool FrontierFinder::isFrontierChanged(const Frontier &ft) {
  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    map_server_->getTSDF()->positionToIndex(cell, idx);
    if (!(knownfree(idx) && isNeighborUnknown(idx)))
      return true;
  }
  return false;
}

void FrontierFinder::computeFrontierInfo(Frontier &ftr) {
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());

  // Compute downsampled cluster
  downsample(ftr.cells_, ftr.filtered_cells_);
}

void FrontierFinder::computeFrontiersToVisit() {
  first_new_ftr_ = frontiers_.end();
  int new_num = 0;
  int new_dormant_num = 0;
  // Try find viewpoints for each cluster and categorize them according to
  // viewpoint number
  for (auto &tmp_ftr : tmp_frontiers_) {
    // Search viewpoints around frontier
    sampleViewpoints(tmp_ftr);
    // sampleViewpoints3D(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty()) {
      ++new_num;
      list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
      // Sort the viewpoints by coverage fraction, best view in front
      sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
           [](const Viewpoint &v1, const Viewpoint &v2) { return v1.visib_num_ > v2.visib_num_; });
      if (first_new_ftr_ == frontiers_.end())
        first_new_ftr_ = inserted;
    } else {
      CHECK_LE(tmp_ftr.visib_num_, min_visib_num_) << "Frontier with valid viewpoint but dormant";
      // Find no viewpoint, move cluster to dormant list
      dormant_frontiers_.push_back(tmp_ftr);
      ++new_dormant_num;

      // remove dormant frontiers, set flag to 0
      for (auto &cell : tmp_ftr.cells_) {
        Eigen::Vector3i idx;
        map_server_->getTSDF()->positionToIndex(cell, idx);
        frontier_flag_[toadr(idx)] = false;
      }
    }
  }

  // Bad implementation, need to be changed to unique id
  // Reset indices of frontiers
  int idx = 0;
  for (auto &ft : frontiers_) {
    ft.id_ = idx++;
    // std::cout << ft.id_ << ", ";
  }
  // std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num
  //           << std::endl;
  // std::cout << "to visit: " << frontiers_.size()
  //           << ", dormant: " << dormant_frontiers_.size() << std::endl;
}

void FrontierFinder::getTopViewpointsInfo(const Vector3d &cur_pos, vector<Eigen::Vector3d> &points,
                                          vector<double> &yaws, vector<Eigen::Vector3d> &averages) {
  points.clear();
  yaws.clear();
  averages.clear();
  for (auto frontier : frontiers_) {
    bool no_view = true;
    for (auto view : frontier.viewpoints_) {
      // Retrieve the first viewpoint that is far enough and has highest
      // coverage
      // min_candidate_dist_ is 1.0 in config file
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_)
        continue;
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      no_view = false;

      // only the first view is pushed back
      break;
    }
    if (no_view) {
      // All viewpoints are very close, just use the first one (with highest
      // coverage).
      auto view = frontier.viewpoints_.front();
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
    }
  }
}

void FrontierFinder::getViewpointsInfo(const Vector3d &cur_pos, const vector<int> &ids,
                                       const int &view_num, const double &max_decay,
                                       vector<vector<Eigen::Vector3d>> &points,
                                       vector<vector<double>> &yaws) {
  points.clear();
  yaws.clear();
  for (auto id : ids) {
    // Scan all frontiers to find one with the same id
    for (auto frontier : frontiers_) {
      if (frontier.id_ == id) {
        // Get several top viewpoints that are far enough
        vector<Eigen::Vector3d> pts;
        vector<double> ys;
        int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
        for (auto view : frontier.viewpoints_) {
          if (pts.size() >= view_num || view.visib_num_ <= visib_thresh)
            break;
          if ((view.pos_ - cur_pos).norm() < min_candidate_dist_)
            continue;
          pts.push_back(view.pos_);
          ys.push_back(view.yaw_);
        }
        if (pts.empty()) {
          // All viewpoints are very close, ignore the distance limit
          for (auto view : frontier.viewpoints_) {
            if (pts.size() >= view_num || view.visib_num_ <= visib_thresh)
              break;
            pts.push_back(view.pos_);
            ys.push_back(view.yaw_);
          }
        }
        points.push_back(pts);
        yaws.push_back(ys);
      }
    }
  }
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>> &clusters) {
  clusters.clear();
  for (auto frontier : frontiers_)
    clusters.push_back(frontier.cells_);
  // clusters.push_back(frontier.filtered_cells_);
}

void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>> &clusters) {
  clusters.clear();
  for (auto ft : dormant_frontiers_)
    clusters.push_back(ft.cells_);
}

void FrontierFinder::getTinyFrontiers(vector<vector<Eigen::Vector3d>> &clusters) {
  clusters.clear();
  for (auto frontier : tiny_frontiers_)
    clusters.push_back(frontier.expended_cells_);
  // clusters.push_back(frontier.filtered_cells_);
}

void FrontierFinder::getFrontiersVisibleNums(vector<int> &visib_nums) {
  visib_nums.clear();
  for (auto frontier : frontiers_)
    visib_nums.push_back(frontier.visib_num_);
}

void FrontierFinder::getDormantFrontiersVisibleNums(vector<int> &visib_nums) {
  visib_nums.clear();
  for (auto frontier : dormant_frontiers_)
    visib_nums.push_back(frontier.visib_num_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &boxes) {
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(std::make_pair(center, scale));
  }
}

void FrontierFinder::getFrontierFirstPCs(vector<Eigen::Vector2d> &pcs) {
  pcs.clear();
  for (auto ftr : frontiers_)
    pcs.push_back(ftr.first_pc_);
}

void FrontierFinder::getUpdateBBox(Vector3d &min, Vector3d &max) {
  min = update_bbox_min_;
  max = update_bbox_max_;
}

// Sample viewpoints around frontier's average position, check coverage to the
// frontier cells
void FrontierFinder::sampleViewpoints(Frontier &frontier) {
  CHECK_EQ(frontier.viewpoints_.size(), 0) << "Frontier already has viewpoints";
  frontier.visib_num_ = 0;

  std::vector<fast_planner::Viewpoint> viewpoints, viewpoints_near_occupied;
  // Evaluate sample viewpoints on circles, find ones that cover most cells
  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
       rc <= candidate_rmax_ + 1e-3; rc += dr) {
    double candidate_dphi = candidate_dphi_;
    if (rc < 1.0)
      candidate_dphi = candidate_dphi_ * 2.0;
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi) {
      const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);

      // Qualified viewpoint is in bounding box and in safe region
      if (!map_server_->isInBox(sample_pos) ||
          map_server_->getOccupancy(sample_pos) == voxel_mapping::OccupancyType::OCCUPIED ||
          map_server_->getOccupancy(sample_pos) == voxel_mapping::OccupancyType::UNKNOWN ||
          isNearUnknown(sample_pos))
        continue;

      // Compute average yaw
      auto &cells = frontier.filtered_cells_;
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      double avg_yaw = 0.0;
      for (int i = 1; i < cells.size(); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        double yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0)
          yaw = -yaw;
        avg_yaw += yaw;
      }
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);

      // Compute the fraction of covered and visible cells
      int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
      if (visib_num > frontier.visib_num_)
        frontier.visib_num_ = visib_num;

      if (visib_num > min_visib_num_) {
        Viewpoint vp = {sample_pos, avg_yaw, visib_num};

        // Check angle with first principal component within [60, 120] (DEPRECATED)
        // frontier first pc does not always follow the alignment of the frontier
        // Eigen::Vector3d dir = vp.pos_ - frontier.average_;
        // double angle =
        //     acos(dir.normalized().dot(Vector3d(frontier.first_pc_[0], frontier.first_pc_[1],
        //     0)));
        // if (angle < M_PI_2 - 0.4 * M_PI || angle > M_PI_2 + 0.4 * M_PI)
        //   continue;

        if (isNearOccupied(sample_pos))
          viewpoints_near_occupied.push_back(vp);
        else
          viewpoints.push_back(vp);

        // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
      }
      // }
    }
  }

  if (viewpoints.empty()) {
    // No qualified viewpoint, use the ones near occupied cells
    for (auto vp : viewpoints_near_occupied)
      viewpoints.push_back(vp);
  }

  if (viewpoints.size() <= 1) {
    frontier.viewpoints_ = viewpoints;
    return;
  }

  // hack code to bypass the viewpoint selection
  // for (int i = 0; i < viewpoints.size(); ++i) {
  //   frontier.viewpoints_.push_back(viewpoints[i]);
  // }
  // return;

  // Check each viewpoint if it covers enough unknown voxels
  vector<int> visib_nums;
  visib_nums.resize(viewpoints.size(), 0);
  for (int i = 0; i < viewpoints.size(); ++i) {
    const Viewpoint &vp = viewpoints[i];
    // cout << "vp: " << vp.pos_.transpose() << ", " << vp.yaw_ << ", " << vp.visib_num_ << endl;
    if (!map_server_->getTSDF()->isInMap(vp.pos_)) {
      continue;
    }

    const vector<Position> &voxels_cam = percep_utils_->cam_fov_voxels_;
    Eigen::Matrix3d R_wb;
    R_wb << cos(vp.yaw_), -sin(vp.yaw_), 0.0, sin(vp.yaw_), cos(vp.yaw_), 0.0, 0.0, 0.0, 1.0;

    // cout << "voxels_cam: " << voxels_cam.size() << endl;
    vector<Vector3d> voxels_world_unknown;
    for (const Position &p : voxels_cam) {

      Position p_w = R_wb * p + vp.pos_;
      if (!map_server_->getTSDF()->isInMap(p_w)) {
        // Find nearest point in box
        p_w = map_server_->getTSDF()->closestPointInMap(p_w, vp.pos_);
      }

      raycaster_->input(vp.pos_, p_w);
      VoxelIndex idx;
      bool blocked = false;
      while (raycaster_->nextId(idx)) {
        if (map_server_->getOccupancyGrid()->isInMap(idx) == false) {
          break;
        }

        if (map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::OCCUPIED) {
          blocked = true;
          break;
        }

        if (map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::UNKNOWN) {
          voxels_world_unknown.push_back(map_server_->getTSDF()->indexToPosition(idx));
        }
      }
      if (blocked)
        continue;
    }

    visib_nums[i] = voxels_world_unknown.size();
    // cout << "voxels_world_unknown: " << voxels_world_unknown.size() << endl;
    //  frontier.viewpoints_.push_back(vp);
  }

  // Calculate normal distribution of visib_nums
  double mean = 0.0;
  for (int i = 0; i < visib_nums.size(); ++i) {
    mean += visib_nums[i];
  }
  mean /= visib_nums.size();
  double sd = 0.0;
  for (int i = 0; i < visib_nums.size(); ++i)
    sd += (visib_nums[i] - mean) * (visib_nums[i] - mean);
  sd = sqrt(sd / visib_nums.size());

  // cout << "mean: " << mean << ", std: " << std << endl;
  // select viewpoints with visib_num > mean
  for (int i = 0; i < visib_nums.size(); ++i) {
    if (visib_nums[i] > mean + viewpoint_z_score_cutoff_ * sd) {
      frontier.viewpoints_.push_back(viewpoints[i]);
    }
  }

  if (viewpoints.size() > 0 && frontier.viewpoints_.size() == 0) {
    frontier.viewpoints_ = viewpoints;
  }
}

void FrontierFinder::sampleViewpoints3D(Frontier &frontier) {
  CHECK_EQ(frontier.viewpoints_.size(), 0) << "Frontier already has viewpoints";

  std::vector<fast_planner::Viewpoint> viewpoints, viewpoints_near_occupied;
  // Evaluate sample viewpoints on cylinders, find ones that cover most cells
  for (double viewpoint_height = frontier.average_.z() - 0.5;
       viewpoint_height <= frontier.average_.z() + 0.51; viewpoint_height += 0.5) {
    for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
         rc <= candidate_rmax_ + 1e-3; rc += dr) {
      double candidate_dphi = candidate_dphi_;
      if (rc < 1.0)
        candidate_dphi = candidate_dphi_ * 2.0;
      for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi) {
        Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);
        sample_pos.z() = viewpoint_height;

        // Qualified viewpoint is in bounding box and in safe region
        if (!map_server_->isInBox(sample_pos) ||
            map_server_->getOccupancy(sample_pos) == voxel_mapping::OccupancyType::OCCUPIED ||
            map_server_->getOccupancy(sample_pos) == voxel_mapping::OccupancyType::UNKNOWN ||
            isNearUnknown(sample_pos))
          continue;

        // Compute average yaw
        auto &cells = frontier.filtered_cells_;
        Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
        double avg_yaw = 0.0;
        for (int i = 1; i < cells.size(); ++i) {
          Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
          double yaw = acos(dir.dot(ref_dir));
          if (ref_dir.cross(dir)[2] < 0)
            yaw = -yaw;
          avg_yaw += yaw;
        }
        avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
        wrapYaw(avg_yaw);

        // Compute the fraction of covered and visible cells
        int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
        if (visib_num > frontier.visib_num_)
          frontier.visib_num_ = visib_num;

        if (visib_num > min_visib_num_) {
          Viewpoint vp = {sample_pos, avg_yaw, visib_num};

          // Check angle with first principal component within [60, 120] (DEPRECATED)
          // frontier first pc does not always follow the alignment of the frontier
          // Eigen::Vector3d dir = vp.pos_ - frontier.average_;
          // double angle =
          //     acos(dir.normalized().dot(Vector3d(frontier.first_pc_[0], frontier.first_pc_[1],
          //     0)));
          // if (angle < M_PI_2 - 0.4 * M_PI || angle > M_PI_2 + 0.4 * M_PI)
          //   continue;

          if (isNearOccupied(sample_pos))
            viewpoints_near_occupied.push_back(vp);
          else
            viewpoints.push_back(vp);
          // frontier.viewpoints_.push_back(vp);

          // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
        }
        // }
      }
    }
  }
  if (viewpoints.empty()) {
    // No qualified viewpoint, use the ones near occupied cells
    for (auto vp : viewpoints_near_occupied)
      viewpoints.push_back(vp);
  }

  // // hack code to bypass the viewpoint selection
  // for (int i = 0; i < viewpoints.size(); ++i) {
  //   frontier.viewpoints_.push_back(viewpoints[i]);
  // }
  // return;

  // Check each viewpoint if it covers enough unknown voxels
  vector<int> visib_nums;
  visib_nums.resize(viewpoints.size(), 0);
  for (int i = 0; i < viewpoints.size(); ++i) {
    const Viewpoint &vp = viewpoints[i];
    // cout << "vp: " << vp.pos_.transpose() << ", " << vp.yaw_ << ", " << vp.visib_num_ << endl;
    if (!map_server_->getTSDF()->isInMap(vp.pos_)) {
      continue;
    }

    const vector<Position> &voxels_cam = percep_utils_->cam_fov_voxels_;
    Eigen::Matrix3d R_wb;
    R_wb << cos(vp.yaw_), -sin(vp.yaw_), 0.0, sin(vp.yaw_), cos(vp.yaw_), 0.0, 0.0, 0.0, 1.0;

    // cout << "voxels_cam: " << voxels_cam.size() << endl;
    vector<Vector3d> voxels_world_unknown;
    for (const Position &p : voxels_cam) {

      Position p_w = R_wb * p + vp.pos_;
      if (!map_server_->getTSDF()->isInMap(p_w)) {
        // Find nearest point in box
        p_w = map_server_->getTSDF()->closestPointInMap(p_w, vp.pos_);
      }

      raycaster_->input(vp.pos_, p_w);
      VoxelIndex idx;
      bool blocked = false;
      while (raycaster_->nextId(idx)) {
        if (map_server_->getOccupancyGrid()->isInMap(idx) == false) {
          // cout << "vp.pos_: " << vp.pos_.transpose() << endl;
          // cout << "p_w: " << p_w.transpose() << endl;
          // cout << "idx pos: " <<
          // map_server_->getTSDF()->indexToPosition(idx).transpose()
          //      << endl;
          // //  raycaster start and end
          // cout << "caster start pos: " << PathCostEvaluator::caster_->getStartPos().transpose()
          // << endl; cout << "caster start idx: " <<
          // PathCostEvaluator::caster_->getStartIndex().transpose() << endl; cout << "caster end
          // pos: " << PathCostEvaluator::caster_->getEndPos().transpose() << endl; cout << "caster
          // end idx: " << PathCostEvaluator::caster_->getEndIndex().transpose() << endl;
          // CHECK(false) << "idx: " << idx.transpose() << endl;
          break;
        }

        if (map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::OCCUPIED) {
          blocked = true;
          break;
        }

        if (map_server_->getOccupancyGrid()->getVoxel(idx).value ==
            voxel_mapping::OccupancyType::UNKNOWN) {
          voxels_world_unknown.push_back(map_server_->getTSDF()->indexToPosition(idx));
        }
      }
      if (blocked)
        continue;
    }

    visib_nums[i] = voxels_world_unknown.size();
    // cout << "voxels_world_unknown: " << voxels_world_unknown.size() << endl;
    //  frontier.viewpoints_.push_back(vp);
  }

  // Calculate normal distribution of visib_nums
  double mean = 0.0;
  for (int i = 0; i < visib_nums.size(); ++i) {
    mean += visib_nums[i];
  }
  mean /= visib_nums.size();
  double sd = 0.0;
  for (int i = 0; i < visib_nums.size(); ++i)
    sd += (visib_nums[i] - mean) * (visib_nums[i] - mean);
  sd = sqrt(sd / visib_nums.size());

  // cout << "mean: " << mean << ", std: " << std << endl;
  // select viewpoints with visib_num > mean
  for (int i = 0; i < visib_nums.size(); ++i) {
    if (visib_nums[i] > mean + viewpoint_z_score_cutoff_ * sd) {
      frontier.viewpoints_.push_back(viewpoints[i]);
    }
  }

  // cout << "viewpoints before: " << viewpoints.size() << endl;
  // cout << "viewpoints after: " << frontier.viewpoints_.size() << endl;
}

bool FrontierFinder::isFrontierCovered() {
  Vector3d update_min, update_max;
  map_server_->getTSDF()->getUpdatedBox(update_min, update_max);

  auto checkChanges = [&](const list<Frontier> &frontiers) {
    for (auto &ftr : frontiers) {
      if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max))
        continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto cell : ftr.cells_) {
        Eigen::Vector3i idx;
        map_server_->getTSDF()->positionToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_))
    return true;

  return false;
}

bool FrontierFinder::isNearUnknown(const Eigen::Vector3d &pos) {
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (map_server_->getOccupancy(vox) == voxel_mapping::OccupancyType::UNKNOWN)
          return true;
      }
  return false;
}

bool FrontierFinder::isNearOccupied(const Eigen::Vector3d &pos) {
  const int vox_num = floor(min_candidate_occupied_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -vox_num; z <= vox_num; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (map_server_->getOccupancy(vox) == voxel_mapping::OccupancyType::OCCUPIED)
          return true;
      }
  return false;
}

int FrontierFinder::countVisibleCells(const Eigen::Vector3d &pos, const double &yaw,
                                      const vector<Eigen::Vector3d> &cluster) {
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  for (auto cell : cluster) {
    // Check if frontier cell is inside FOV
    if (!percep_utils_->insideFOV(cell))
      continue;

    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      if (map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::OCCUPIED ||
          map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::UNKNOWN) {
        visib = false;
        break;
      }
    }
    if (visib)
      visib_num += 1;
  }
  return visib_num;
}

void FrontierFinder::downsample(const vector<Eigen::Vector3d> &cluster_in,
                                vector<Eigen::Vector3d> &cluster_out) {
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const double leaf_size = map_server_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  // find the nearest voxel in the downsampled cluster
  for (auto &pt : cloudf->points) {
    VoxelIndex idx;
    Eigen::Vector3d pos(pt.x, pt.y, pt.z);
    map_server_->getTSDF()->positionToIndex(pos, idx);
    Position pos_out;
    map_server_->getTSDF()->indexToPosition(idx, pos_out);
    pt.x = pos_out[0];
    pt.y = pos_out[1];
    pt.z = pos_out[2];
  }

  cluster_out.clear();
  for (auto pt : cloudf->points)
    cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double &yaw) {
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}

inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i &voxel) {
  vector<Eigen::Vector3i> neighbors(6);
  Eigen::Vector3i tmp;

  tmp = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[0] = tmp;
  tmp = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[1] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[2] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[3] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[4] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 0, 1);
  neighbors[5] = tmp;

  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i &voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0)
        continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i &voxel) {
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0)
          continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i &voxel) {
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (map_server_->getOccupancy(nbr) == voxel_mapping::OccupancyType::UNKNOWN)
      return true;
  }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i &idx) {
  return map_server_->getTSDF()->indexToAddress(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i &idx) {
  return map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::FREE;
}

void FrontierFinder::setCellHeights(const vector<double> &cell_heights) {
  cell_heights_ = cell_heights;
}

} // namespace fast_planner