#include <pcl/kdtree/kdtree_flann.h>

#include "exploration_preprocessing/hierarchical_grid.h"

namespace fast_planner {

UniformGrid::UniformGrid() {}

UniformGrid::~UniformGrid() {}

UniformGrid::UniformGrid(ros::NodeHandle &nh, const Config &config,
                         voxel_mapping::MapServer::Ptr &map_server)
    : config_(config), map_server_(map_server) {
  // Initialize uniform grid
  uniform_grid_.resize(config_.num_cells_x_ * config_.num_cells_y_ * config_.num_cells_z_);

  // Initialize each grid cell
  for (unsigned int i = 0; i < uniform_grid_.size(); ++i) {
    uniform_grid_[i].id_ = i;

    int x = i % config_.num_cells_x_;
    int y = (i / config_.num_cells_x_) % config_.num_cells_y_;
    int z = i / (config_.num_cells_x_ * config_.num_cells_y_);

    uniform_grid_[i].bbox_min_.x() = config_.bbox_min_.x() + x * config_.cell_size_.x();
    uniform_grid_[i].bbox_min_.y() = config_.bbox_min_.y() + y * config_.cell_size_.y();
    uniform_grid_[i].bbox_min_.z() = config_.bbox_min_.z() + z * config_.cell_size_.z();

    uniform_grid_[i].bbox_max_.x() = config_.bbox_min_.x() + (x + 1) * config_.cell_size_.x();
    uniform_grid_[i].bbox_max_.y() = config_.bbox_min_.y() + (y + 1) * config_.cell_size_.y();
    uniform_grid_[i].bbox_max_.z() = config_.bbox_min_.z() + (z + 1) * config_.cell_size_.z();

    // Check if the cell inside the box bounding box
    if (!map_server_->getTSDF()->isInBox(uniform_grid_[i].bbox_min_) ||
        !map_server_->getTSDF()->isInBox(uniform_grid_[i].bbox_max_)) {
      Position box_bbox_min, box_bbox_max;
      map_server_->getTSDF()->getBoxBoundingBox(box_bbox_min, box_bbox_max);
      // ROS_WARN("[UniformGrid] Cell %d is outside the box bounding box", i);
      for (int j = 0; j < 3; j++) {
        if (uniform_grid_[i].bbox_min_(j) < box_bbox_min(j)) {
          uniform_grid_[i].bbox_min_(j) = box_bbox_min(j);
        }
        if (uniform_grid_[i].bbox_max_(j) > box_bbox_max(j)) {
          uniform_grid_[i].bbox_max_(j) = box_bbox_max(j);
        }
      }
    }

    uniform_grid_[i].bbox_min_idx_ =
        map_server_->getTSDF()->positionToIndex(uniform_grid_[i].bbox_min_);
    uniform_grid_[i].bbox_max_idx_ =
        map_server_->getTSDF()->positionToIndex(uniform_grid_[i].bbox_max_);

    uniform_grid_[i].center_ = (uniform_grid_[i].bbox_min_ + uniform_grid_[i].bbox_max_) / 2.0;
    uniform_grid_[i].center_free_ = uniform_grid_[i].center_;
    uniform_grid_[i].centers_unknown_.push_back(uniform_grid_[i].center_);
    uniform_grid_[i].centers_unknown_active_.push_back(uniform_grid_[i].center_);
    uniform_grid_[i].centers_unknown_active_idx_.push_back(0);

    uniform_grid_[i].nearby_cells_ids_.clear();
    if (x > 0) {
      uniform_grid_[i].nearby_cells_ids_.push_back(i - 1);
    }
    if (x < config_.num_cells_x_ - 1) {
      uniform_grid_[i].nearby_cells_ids_.push_back(i + 1);
    }
    if (y > 0) {
      uniform_grid_[i].nearby_cells_ids_.push_back(i - config_.num_cells_x_);
    }
    if (y < config_.num_cells_y_ - 1) {
      uniform_grid_[i].nearby_cells_ids_.push_back(i + config_.num_cells_x_);
    }
    if (z > 0) {
      uniform_grid_[i].nearby_cells_ids_.push_back(i - config_.num_cells_x_ * config_.num_cells_y_);
    }
    if (z < config_.num_cells_z_ - 1) {
      uniform_grid_[i].nearby_cells_ids_.push_back(i + config_.num_cells_x_ * config_.num_cells_y_);
    }
    // ROS_INFO("[UniformGrid] Cell %d has %d nearby cells", i,
    // uniform_grid_[i].nearby_cells_ids_.size()); std::cout << "[UniformGrid] Cell " << i << "
    // nearby cells: "; for (int j = 0; j < uniform_grid_[i].nearby_cells_ids_.size(); ++j) {
    //   std::cout << uniform_grid_[i].nearby_cells_ids_[j] << " ";
    // }
    // std::cout << std::endl;

    uniform_grid_[i].unknown_num_ = config_.num_voxels_per_cell_.prod(); // All unknown at first
    uniform_grid_[i].frontier_num_ = 0;                                  // No frontier at first
    uniform_grid_[i].frontier_ids_.clear();

    // Get 8 vertices of the cell
    Position p1, p2, p3, p4, p5, p6, p7, p8;
    p1 = Position(uniform_grid_[i].bbox_min_.x(), uniform_grid_[i].bbox_min_.y(),
                  uniform_grid_[i].bbox_min_.z());
    p2 = Position(uniform_grid_[i].bbox_max_.x(), uniform_grid_[i].bbox_min_.y(),
                  uniform_grid_[i].bbox_min_.z());
    p3 = Position(uniform_grid_[i].bbox_max_.x(), uniform_grid_[i].bbox_max_.y(),
                  uniform_grid_[i].bbox_min_.z());
    p4 = Position(uniform_grid_[i].bbox_min_.x(), uniform_grid_[i].bbox_max_.y(),
                  uniform_grid_[i].bbox_min_.z());
    p5 = Position(uniform_grid_[i].bbox_min_.x(), uniform_grid_[i].bbox_min_.y(),
                  uniform_grid_[i].bbox_max_.z());
    p6 = Position(uniform_grid_[i].bbox_max_.x(), uniform_grid_[i].bbox_min_.y(),
                  uniform_grid_[i].bbox_max_.z());
    p7 = Position(uniform_grid_[i].bbox_max_.x(), uniform_grid_[i].bbox_max_.y(),
                  uniform_grid_[i].bbox_max_.z());
    p8 = Position(uniform_grid_[i].bbox_min_.x(), uniform_grid_[i].bbox_max_.y(),
                  uniform_grid_[i].bbox_max_.z());

    // Get lines of each side of the cell
    // vertices1_ and vertices2_ saves the two ends of each line
    // z        y
    //     8------7
    //   / |     /|
    //  /  |    / |
    // 5-------6  |
    // |   4---|--3
    // |  /    |  /
    // | /     | /
    // |/      |/
    // 1-------2   x
    uniform_grid_[i].vertices1_.push_back(p1);
    uniform_grid_[i].vertices1_.push_back(p2);
    uniform_grid_[i].vertices1_.push_back(p3);
    uniform_grid_[i].vertices1_.push_back(p1);
    uniform_grid_[i].vertices1_.push_back(p1);
    uniform_grid_[i].vertices1_.push_back(p4);
    uniform_grid_[i].vertices1_.push_back(p3);
    uniform_grid_[i].vertices1_.push_back(p2);
    uniform_grid_[i].vertices1_.push_back(p5);
    uniform_grid_[i].vertices1_.push_back(p5);
    uniform_grid_[i].vertices1_.push_back(p6);
    uniform_grid_[i].vertices1_.push_back(p7);

    uniform_grid_[i].vertices2_.push_back(p2);
    uniform_grid_[i].vertices2_.push_back(p3);
    uniform_grid_[i].vertices2_.push_back(p4);
    uniform_grid_[i].vertices2_.push_back(p4);
    uniform_grid_[i].vertices2_.push_back(p5);
    uniform_grid_[i].vertices2_.push_back(p8);
    uniform_grid_[i].vertices2_.push_back(p7);
    uniform_grid_[i].vertices2_.push_back(p6);
    uniform_grid_[i].vertices2_.push_back(p6);
    uniform_grid_[i].vertices2_.push_back(p8);
    uniform_grid_[i].vertices2_.push_back(p7);
    uniform_grid_[i].vertices2_.push_back(p8);

    if (config_.verbose_)
      uniform_grid_[i].print();
  }

  // Init cost matrix
  cost_matrix_ = std::vector<std::vector<double>>(
      uniform_grid_.size(), std::vector<double>(uniform_grid_.size(), 1000.0));
  for (unsigned int i = 0; i < uniform_grid_.size(); ++i) {
    cost_matrix_[i][i] = 0.0;
    for (unsigned int j = 0; j < uniform_grid_[i].nearby_cells_ids_.size(); ++j) {
      cost_matrix_[i][uniform_grid_[i].nearby_cells_ids_[j]] =
          (uniform_grid_[i].center_ - uniform_grid_[uniform_grid_[i].nearby_cells_ids_[j]].center_)
              .norm() /
          PathCostEvaluator::vm_;

      // Add bias on X axis to avoid zigzag
      cost_matrix_[i][uniform_grid_[i].nearby_cells_ids_[j]] +=
          0.01 * fabs(uniform_grid_[i].center_.x() -
                      uniform_grid_[uniform_grid_[i].nearby_cells_ids_[j]].center_.x());
    }
  }

  // std::cout << "[UniformGrid] Connectivity matrix: " << std::endl;

  // To save voxels in each CCL zones of the cell
  ccl_voxels_addr_.resize(uniform_grid_.size());
  ccl_voxels_color_.resize(uniform_grid_.size());
  ccl_free_unknown_states_and_centers_idx_.resize(uniform_grid_.size());

  for (unsigned int i = 0; i < uniform_grid_.size(); ++i) {
    Position bbox_min = uniform_grid_[i].bbox_min_;
    Position bbox_max = uniform_grid_[i].bbox_max_;
    bbox_min.z() = 1.0;
    bbox_max.z() = 1.0;

    VoxelIndex bbox_min_idx = map_server_->getTSDF()->positionToIndex(bbox_min);
    VoxelIndex bbox_max_idx = map_server_->getTSDF()->positionToIndex(bbox_max);

    std::unordered_set<int> addrs;
    int z = bbox_min_idx.z();
    for (int x = bbox_min_idx.x(); x <= bbox_max_idx.x(); x += config_.ccl_step_) {
      for (int y = bbox_min_idx.y(); y <= bbox_max_idx.y(); y += config_.ccl_step_) {
        VoxelIndex idx(x, y, z);
        int addr = map_server_->getTSDF()->indexToAddress(idx);
        if (map_server_->getTSDF()->isInBox(map_server_->getTSDF()->indexToPosition(idx))) {
          addrs.insert(addr);
        }
      }
    }
    ccl_voxels_addr_[i].push_back(addrs);
    ccl_voxels_color_[i].push_back(Eigen::Vector3d(230.0 / 255.0, 230.0 / 255.0, 230.0 / 255.0));
    ccl_free_unknown_states_and_centers_idx_[i].push_back(std::make_pair(1, 0));
  }

  // Init connectivity graph
  connectivity_graph_ = std::make_shared<ConnectivityGraph>(nh);
  for (unsigned int i = 0; i < uniform_grid_.size(); ++i) {
    ConnectivityNode::Ptr node = std::make_shared<ConnectivityNode>(
        uniform_grid_[i].id_ * 10, uniform_grid_[i].center_, ConnectivityNode::TYPE::UNKNOWN);
    for (unsigned int j = 0; j < uniform_grid_[i].nearby_cells_ids_.size(); ++j) {
      node->addNeighbor(uniform_grid_[i].nearby_cells_ids_[j] * 10,
                        cost_matrix_[i][uniform_grid_[i].nearby_cells_ids_[j]],
                        ConnectivityEdge::TYPE::UNKNOWN);
    }
    connectivity_graph_->addNode(node);
  }

  ROS_INFO("[UniformGrid] Uniforma grid level %d initialization finished", config_.level_);
}

int UniformGrid::positionToGridCellId(const Position &pos) {
  int x = std::floor((pos.x() - config_.bbox_min_.x()) / config_.cell_size_.x());
  int y = std::floor((pos.y() - config_.bbox_min_.y()) / config_.cell_size_.y());
  int z = std::floor((pos.z() - config_.bbox_min_.z()) / config_.cell_size_.z());

  return x + y * config_.num_cells_x_ + z * config_.num_cells_x_ * config_.num_cells_y_;
}

void UniformGrid::positionToGridCellId(const Position &pos, int &id) {
  id = positionToGridCellId(pos);
}

void UniformGrid::positionToGridCellCenterId(const Position &pos, int &cell_id, int &center_id) {
  cell_id = -1;
  center_id = -1;

  cell_id = positionToGridCellId(pos);

  Position bbox_min = uniform_grid_[cell_id].bbox_min_;
  Position bbox_max = uniform_grid_[cell_id].bbox_max_;

  VoxelIndex bbox_min_idx = map_server_->getTSDF()->positionToIndex(bbox_min);
  VoxelIndex bbox_max_idx = map_server_->getTSDF()->positionToIndex(bbox_max);

  Eigen::Vector3i idx = map_server_->getTSDF()->positionToIndex(pos) - bbox_min_idx;

  // Find nearest idx with ccl_step_
  Eigen::Vector3i idx_nearest;
  idx_nearest.x() = std::floor(idx.x() / config_.ccl_step_) * config_.ccl_step_;
  idx_nearest.y() = std::floor(idx.y() / config_.ccl_step_) * config_.ccl_step_;
  idx_nearest.z() = std::floor(idx.z() / config_.ccl_step_) * config_.ccl_step_;

  int addr = map_server_->getTSDF()->indexToAddress(idx_nearest + bbox_min_idx);
  const std::vector<std::pair<int, int>> &free_unknown_states_and_centers_idx =
      ccl_free_unknown_states_and_centers_idx_[cell_id];
  for (int j = 0; j < ccl_voxels_addr_[cell_id].size(); ++j) {
    std::unordered_set<int> &voxel_addrs = ccl_voxels_addr_[cell_id][j];
    if (voxel_addrs.find(addr) != voxel_addrs.end()) {
      if (free_unknown_states_and_centers_idx[j].first == 0) {
        center_id = free_unknown_states_and_centers_idx[j].second;
      } else {
        center_id = free_unknown_states_and_centers_idx[j].second +
                    uniform_grid_[cell_id].centers_free_.size();
      }
      break;
    }
  }

  if (cell_id != -1 && center_id != -1) {
    // Find cell id and center id, return
    return;
  }

  // If not found, find cell id and center id using local astar search

  // ROS_INFO("[UniformGrid] Position (%f, %f, %f) in cell %d, center %d", pos.x(), pos.y(),
  // pos.z(),
  //          cell_id, center_id);

  // CHECK_GE(cell_id, 0) << "Invalid cell id in positionToGridCellCenterId";
  // CHECK_GE(center_id, 0) << "Invalid center id in positionToGridCellCenterId";
}

void UniformGrid::getGridCellIdsInBBox(const Position &bbox_min, const Position &bbox_max,
                                       std::vector<int> &cell_ids) {
  // limite x, y, z between config_.bbox_min_ and config_.bbox_max_
  Position bbox_min_limited = bbox_min;
  Position bbox_max_limited = bbox_max;
  for (int i = 0; i < 3; i++) {
    if (bbox_min_limited(i) < config_.bbox_min_(i)) {
      bbox_min_limited(i) = config_.bbox_min_(i);
    }
    if (bbox_max_limited(i) > config_.bbox_max_(i)) {
      bbox_max_limited(i) = config_.bbox_max_(i);
    }
  }

  int x_min = std::floor((bbox_min_limited.x() - config_.bbox_min_.x()) / config_.cell_size_.x());
  int y_min = std::floor((bbox_min_limited.y() - config_.bbox_min_.y()) / config_.cell_size_.y());
  int z_min = std::floor((bbox_min_limited.z() - config_.bbox_min_.z()) / config_.cell_size_.z());
  int x_max = std::floor((bbox_max_limited.x() - config_.bbox_min_.x()) / config_.cell_size_.x());
  int y_max = std::floor((bbox_max_limited.y() - config_.bbox_min_.y()) / config_.cell_size_.y());
  int z_max = std::floor((bbox_max_limited.z() - config_.bbox_min_.z()) / config_.cell_size_.z());

  for (int x = x_min; x <= x_max; ++x) {
    for (int y = y_min; y <= y_max; ++y) {
      for (int z = z_min; z <= z_max; ++z) {
        int id = x + y * config_.num_cells_x_ + z * config_.num_cells_x_ * config_.num_cells_y_;
        if (id < 0 || id > config_.num_cells_ - 1) {
          if (config_.verbose_)
            ROS_WARN("[UniformGrid] Invalid cell id %d", id);

          continue;
        }
        cell_ids.push_back(id);
      }
    }
  }
}

void UniformGrid::inputFrontiers(const std::vector<Position> &frontier_viewpoints,
                                 const std::vector<double> &frontier_yaws) {
  CHECK_EQ(frontier_viewpoints.size(), frontier_yaws.size()) << "Frontier size mismatch";
  // ROS_INFO("[UniformGrid] Input %lu frontiers", frontier_viewpoints.size());

  // Record previous frontier info
  std::vector<int> previous_frontier_nums(uniform_grid_.size(), 0);
  std::vector<std::vector<int>> previous_frontier_ids(uniform_grid_.size());
  std::vector<std::vector<Position>> previous_frontier_viewpoints(uniform_grid_.size());
  std::vector<std::vector<double>> previous_frontier_yaws(uniform_grid_.size());
  for (int i = 0; i < config_.num_cells_; i++) {
    previous_frontier_nums[i] = uniform_grid_[i].frontier_num_;
    previous_frontier_ids[i] = uniform_grid_[i].frontier_ids_;
    previous_frontier_viewpoints[i] = uniform_grid_[i].frontier_viewpoints_;
    previous_frontier_yaws[i] = uniform_grid_[i].frontier_yaws_;

    // Clear frontier info in each cell
    uniform_grid_[i].frontier_num_ = 0;
    uniform_grid_[i].frontier_ids_.clear();
    uniform_grid_[i].frontier_viewpoints_.clear();
    uniform_grid_[i].frontier_yaws_.clear();

    // Clear frontier info in each free subspace
    // uniform_grid_[i].frontier_ids_mc_.clear();
    // uniform_grid_[i].frontier_viewpoints_mc_.clear();
    // uniform_grid_[i].frontier_yaws_mc_.clear();
  }

  for (int frontier_id = 0; frontier_id < frontier_viewpoints.size(); frontier_id++) {
    Position frontier_viewpoint = frontier_viewpoints[frontier_id];
    double frontiers_yaw = frontier_yaws[frontier_id];

    int cell_id = positionToGridCellId(frontier_viewpoint);
    // ROS_INFO("[UniformGrid] Frontier %d in cell %d, cell state: %d", frontier_id, cell_id,
    //          uniform_grid_[cell_id].state_);

    if (cell_id < 0 || cell_id > config_.num_cells_ - 1) {
      ROS_WARN("[UniformGrid] Invalid cell id %d, frontier_id: %d", cell_id, frontier_id);
      CHECK(false) << "Invalid cell id in inputFrontiers. frontier_id: " << frontier_id
                   << ", frontier_viewpoint: " << frontier_viewpoint.transpose();
    }

    {
      // Might rediscover some explored cells, add it back to active cells
      // if (uniform_grid_[cell_id].state_ == GridCell::STATE::EXPLORED) {
      //   uniform_grid_[cell_id].state_ = GridCell::STATE::ACTIVE;
      //   active_cell_ids_.push_back(cell_id);
      // }

      // If not in active cells, add it to active cells
      // if (std::find(active_cell_ids_.begin(), active_cell_ids_.end(), cell_id) ==
      //     active_cell_ids_.end()) {
      //   active_cell_ids_.push_back(cell_id);
      // }
    }

    uniform_grid_[cell_id].frontier_num_++;
    uniform_grid_[cell_id].frontier_ids_.push_back(frontier_id);
    uniform_grid_[cell_id].frontier_viewpoints_.push_back(frontier_viewpoint);
    uniform_grid_[cell_id].frontier_yaws_.push_back(frontiers_yaw);
  }

  // Hack code for bad frontier id implementation in FUEL
  // It is recommended to use change frontier id to unique id in the future
  // If num changed, update cell (CCL decomposition)
  // If num not changed, compare each frontier viewpoint and yaw
  // If viewpoint or yaw changed, update cell (CCL decomposition)
  cell_ids_need_update_frontier_.clear();
  for (int i = 0; i < config_.num_cells_; i++) {
    if (uniform_grid_[i].frontier_num_ == 0 && previous_frontier_nums[i] == 0) {
      // No frontier in this cell, no need to update
      // ROS_INFO("[UniformGrid] Cell %d has no frontiers", i);
      continue;
    }

    if (uniform_grid_[i].frontier_num_ == 0 && previous_frontier_nums[i] != 0) {
      // ROS_INFO("[UniformGrid] Cell %d has no frontiers, but previous has %d frontiers", i,
      //          previous_frontier_nums[i]);
      // No frontier in this cell, clear previous frontier info
      uniform_grid_[i].centers_free_active_.clear();
      uniform_grid_[i].frontier_ids_mc_.clear();
      uniform_grid_[i].frontier_viewpoints_mc_.clear();
      uniform_grid_[i].frontier_yaws_mc_.clear();
    }

    if (uniform_grid_[i].frontier_num_ != previous_frontier_nums[i]) {
      // ROS_INFO("[UniformGrid] Cell %d has %d frontiers, previous has %d frontiers, needs update",
      // i,
      //          uniform_grid_[i].frontier_num_, previous_frontier_nums[i]);
      // Frontier num changed, update cell
      cell_ids_need_update_frontier_.push_back(i);
    } else {
      // ROS_INFO("[UniformGrid] Cell %d has %d frontiers, same as previous, needs compare", i,
      //          uniform_grid_[i].frontier_num_, previous_frontier_nums[i]);
      // Frontier num not changed, compare each frontier viewpoint and yaw
      std::map<int, int> frontier_id_map_previous_to_new;
      for (int j = 0; j < uniform_grid_[i].frontier_num_; j++) {
        Position frontier_viewpoint = uniform_grid_[i].frontier_viewpoints_[j];
        double frontiers_yaw = uniform_grid_[i].frontier_yaws_[j];

        int new_frontier_id = uniform_grid_[i].frontier_ids_[j];
        int previous_frontier_id = -1;

        bool viewpoint_changed = true;
        for (int k = 0; k < previous_frontier_nums[i]; k++) {
          if ((frontier_viewpoint - previous_frontier_viewpoints[i][k]).norm() < 1e-3 &&
              fabs(frontiers_yaw - previous_frontier_yaws[i][k]) < 1e-3) {
            viewpoint_changed = false;
            previous_frontier_id = previous_frontier_ids[i][k];
            break;
          }
        }

        if (viewpoint_changed) {
          // Viewpoint changed, update cell
          // ROS_INFO("[UniformGrid] Cell %d frontier viewpoint changed, needs update", i);
          cell_ids_need_update_frontier_.push_back(i);
          break;
        }

        if (new_frontier_id != previous_frontier_id) {
          // Frontier id changed, update cell
          // ROS_WARN("[UniformGrid] Cell %d frontier id changed, new id %d, previous id %d, only "
          //          "needs update id",
          //  i, new_frontier_id, previous_frontier_id);
          frontier_id_map_previous_to_new[previous_frontier_id] = new_frontier_id;
        }
        // else {
        // ROS_INFO("[UniformGrid] Cell %d frontier new id %d, previous id %d, same", i,
        //          new_frontier_id, previous_frontier_id);
        // }
      }

      if (frontier_id_map_previous_to_new.size() > 0) {
        // print uniform_grid_[i].frontier_ids_mc_
        for (const std::vector<int> &frontier_ids : uniform_grid_[i].frontier_ids_mc_) {
          string frontier_ids_mc_string;
          for (int frontier_id : frontier_ids) {
            frontier_ids_mc_string += std::to_string(frontier_id) + " ";
          }
          // ROS_INFO("[UniformGrid] Cell %d frontier ids mc: %s", i,
          // frontier_ids_mc_string.c_str());
        }

        for (std::vector<int> &frontier_ids : uniform_grid_[i].frontier_ids_mc_) {
          for (int &frontier_id : frontier_ids) {
            if (frontier_id_map_previous_to_new.find(frontier_id) !=
                frontier_id_map_previous_to_new.end()) {
              frontier_id = frontier_id_map_previous_to_new[frontier_id];
            }
          }
        }

        for (const std::vector<int> &frontier_ids : uniform_grid_[i].frontier_ids_mc_) {
          string frontier_ids_mc_string;
          for (int frontier_id : frontier_ids) {
            frontier_ids_mc_string += std::to_string(frontier_id) + " ";
          }
          // ROS_INFO("[UniformGrid] Cell %d frontier ids mc: %s", i,
          // frontier_ids_mc_string.c_str());
        }
      }
    }
  }

  // Update state if the frontier viewpoint in the cell is removed
  // for (int i = 0; i < config_.num_cells_; i++) {
  //   if (uniform_grid_[i].state_ == GridCell::STATE::ACTIVE &&
  //       uniform_grid_[i].unknown_num_ < config_.min_unknown_num_ && previous_frontier_nums[i] > 0
  //       && uniform_grid_[i].frontier_num_ == 0) {
  //     ROS_INFO("[UniformGrid] Cell %d has no frontiers after update, change state to EXPLORED",
  //     i); uniform_grid_[i].state_ = GridCell::STATE::EXPLORED;

  //     active_cell_ids_.erase(std::remove(active_cell_ids_.begin(), active_cell_ids_.end(), i),
  //                            active_cell_ids_.end());
  //   }
  // }

  // Print frontier ids in each cell
  if (config_.verbose_) {
    for (int i = 0; i < config_.num_cells_; i++) {
      if (previous_frontier_nums[i] > 0) {
        std::string previous_frontier_id_string;
        for (int previous_frontier_id : previous_frontier_ids[i]) {
          previous_frontier_id_string += std::to_string(previous_frontier_id) + " ";
        }
        ROS_INFO("[UniformGrid] Cell %d has %d previous frontiers: %s", i,
                 previous_frontier_nums[i], previous_frontier_id_string.c_str());
      }

      if (uniform_grid_[i].frontier_num_ > 0) {
        std::string frontier_id_string;
        for (auto it = uniform_grid_[i].frontier_ids_.begin();
             it != uniform_grid_[i].frontier_ids_.end(); ++it) {
          frontier_id_string += std::to_string(*it) + " ";
        }
        ROS_INFO("[UniformGrid] Cell %d has %d frontiers: %s", i, uniform_grid_[i].frontier_num_,
                 frontier_id_string.c_str());
      }
    }

    std::string cell_ids_need_update_string;
    for (int cell_id : cell_ids_need_update_frontier_) {
      cell_ids_need_update_string += std::to_string(cell_id) + " ";
    }
    ROS_INFO("[UniformGrid] %lu cells need update: %s", cell_ids_need_update_frontier_.size(),
             cell_ids_need_update_string.c_str());
  }
}

void UniformGrid::updateGridsFromVoxelMap(const Position &update_bbox_min,
                                          const Position &update_bbox_max) {
  // Get update bounding box
  Position bbox_min = update_bbox_min;
  Position bbox_max = update_bbox_max;

  if (config_.verbose_)
    ROS_INFO("[UniformGrid] Update bounding box: (%f, %f, %f) - (%f, %f, %f)", bbox_min.x(),
             bbox_min.y(), bbox_min.z(), bbox_max.x(), bbox_max.y(), bbox_max.z());

  // Get the cells that have overlaps the update bounding box
  std::vector<int> update_cell_ids;
  getGridCellIdsInBBox(bbox_min, bbox_max, update_cell_ids);

  if (config_.verbose_) {
    string update_cell_id_string;
    for (int update_cell_id : update_cell_ids) {
      update_cell_id_string += std::to_string(update_cell_id) + " ";
    }
    ROS_INFO("[UniformGrid] Update %lu cells: %s", update_cell_ids.size(),
             update_cell_id_string.c_str());
  }

  ros::Time t1;
  t1 = ros::Time::now();
  for (const int &update_cell_id : update_cell_ids) {
    Position cell_min, cell_max;
    cell_min = uniform_grid_[update_cell_id].bbox_min_;
    cell_max = uniform_grid_[update_cell_id].bbox_max_;

    // Delete nodes in connectivity graph
    // centers_free_ and centers_unknown_ here are previous centers
    int total_center_num = uniform_grid_[update_cell_id].centers_free_.size() +
                           uniform_grid_[update_cell_id].centers_unknown_.size();
    for (int i = 0; i < total_center_num; i++) {
      connectivity_graph_->removeNode(uniform_grid_[update_cell_id].id_ * 10 + i);
    }

    uniform_grid_[update_cell_id].centers_free_.clear();
    uniform_grid_[update_cell_id].centers_free_active_.clear();
    uniform_grid_[update_cell_id].centers_unknown_.clear();
    uniform_grid_[update_cell_id].centers_unknown_active_.clear();

    // save the active center idx in centers_free_ and centers_unknown_
    uniform_grid_[update_cell_id].centers_free_active_idx_.clear();
    uniform_grid_[update_cell_id].centers_unknown_active_idx_.clear();

    // Ver 2.0 center position (multiple centers for unknown and free spaces)
    // Using CCL algorithm to get free centers and unknown centers
    if (config_.use_2d_grids_)
      getCCLCenters2D(update_cell_id, uniform_grid_[update_cell_id].centers_free_,
                      uniform_grid_[update_cell_id].centers_unknown_);
    else
      getCCLCenters(update_cell_id, uniform_grid_[update_cell_id].centers_free_,
                    uniform_grid_[update_cell_id].centers_unknown_);

    // centers_unknown_active_ = centers_unknown_ by default
    // inactive unknown centers will be removed later
    uniform_grid_[update_cell_id].centers_unknown_active_ =
        uniform_grid_[update_cell_id].centers_unknown_;
    for (int i = 0; i < uniform_grid_[update_cell_id].centers_unknown_.size(); i++) {
      uniform_grid_[update_cell_id].centers_unknown_active_idx_.push_back(i);
    }

    // Init nodes and add them to connectivity graph
    // Add centers_free_ to connectivity graph
    for (int i = 0; i < uniform_grid_[update_cell_id].centers_free_.size(); i++) {
      Position center = uniform_grid_[update_cell_id].centers_free_[i];
      ConnectivityNode::Ptr node = std::make_shared<ConnectivityNode>(
          uniform_grid_[update_cell_id].id_ * 10 + i, center, ConnectivityNode::TYPE::FREE);
      connectivity_graph_->addNode(node);
    }

    // Add centers_unknown_ to connectivity graph
    for (int i = 0; i < uniform_grid_[update_cell_id].centers_unknown_.size(); i++) {
      Position center = uniform_grid_[update_cell_id].centers_unknown_[i];
      ConnectivityNode::Ptr node = std::make_shared<ConnectivityNode>(
          uniform_grid_[update_cell_id].id_ * 10 +
              uniform_grid_[update_cell_id].centers_free_.size() + i,
          center, ConnectivityNode::TYPE::UNKNOWN);
      connectivity_graph_->addNode(node);
    }

    if (config_.verbose_) {
      ROS_INFO("[UniformGrid] Cell %d Unknown voxel num: %d", update_cell_id,
               uniform_grid_[update_cell_id].unknown_num_);
      ROS_INFO("[UniformGrid] Center: (%f, %f, %f)", uniform_grid_[update_cell_id].center_.x(),
               uniform_grid_[update_cell_id].center_.y(),
               uniform_grid_[update_cell_id].center_.z());
    }
  }

  space_decomp_time_ = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Add neighbors relationship to connectivity graph
  for (const int &update_cell_id : update_cell_ids) {
    // current cell's centers_free_ can be connected with current cells' centers_unknown_ (PORTAL)
    // and nearby cells' centers_free_ (FREE)
    for (int i = 0; i < uniform_grid_[update_cell_id].centers_free_.size(); i++) {
      // Add current cell's centers_unknown_ as neighbors
      ConnectivityNode::Ptr node =
          connectivity_graph_->getNode(uniform_grid_[update_cell_id].id_ * 10 + i);
      CHECK_NOTNULL(node.get());

      for (int j = 0; j < uniform_grid_[update_cell_id].centers_unknown_.size(); j++) {
        ConnectivityNode::Ptr node_neighbor =
            connectivity_graph_->getNode(uniform_grid_[update_cell_id].id_ * 10 +
                                         uniform_grid_[update_cell_id].centers_free_.size() + j);
        CHECK_NOTNULL(node_neighbor.get());

        std::vector<Position> path, reverse_path;
        double cost = getAStarCostBBoxUnknown(uniform_grid_[update_cell_id].centers_free_[i],
                                              uniform_grid_[update_cell_id].centers_unknown_[j],
                                              uniform_grid_[update_cell_id].bbox_min_,
                                              uniform_grid_[update_cell_id].bbox_max_, path);

        // reverse path
        for (int i = path.size() - 1; i >= 0; i--) {
          reverse_path.push_back(path[i]);
        }

        node->addNeighborWithPath(uniform_grid_[update_cell_id].id_ * 10 +
                                      uniform_grid_[update_cell_id].centers_free_.size() + j,
                                  cost, ConnectivityEdge::TYPE::PORTAL, path);
        node_neighbor->addNeighborWithPath(uniform_grid_[update_cell_id].id_ * 10 + i, cost,
                                           ConnectivityEdge::TYPE::PORTAL, reverse_path);
      }

      // Add nearby cells' centers_free_ as neighbors
      for (int nearby_cell_id : uniform_grid_[update_cell_id].nearby_cells_ids_) {
        for (int k = 0; k < uniform_grid_[nearby_cell_id].centers_free_.size(); k++) {
          ConnectivityNode::Ptr node_neighbor =
              connectivity_graph_->getNode(uniform_grid_[nearby_cell_id].id_ * 10 + k);
          CHECK_NOTNULL(node_neighbor.get());

          Position local_astar_bbox_min, local_astar_bbox_max;
          local_astar_bbox_min = Position(min(uniform_grid_[update_cell_id].bbox_min_.x(),
                                              uniform_grid_[nearby_cell_id].bbox_min_.x()),
                                          min(uniform_grid_[update_cell_id].bbox_min_.y(),
                                              uniform_grid_[nearby_cell_id].bbox_min_.y()),
                                          min(uniform_grid_[update_cell_id].bbox_min_.z(),
                                              uniform_grid_[nearby_cell_id].bbox_min_.z()));
          local_astar_bbox_max = Position(max(uniform_grid_[update_cell_id].bbox_max_.x(),
                                              uniform_grid_[nearby_cell_id].bbox_max_.x()),
                                          max(uniform_grid_[update_cell_id].bbox_max_.y(),
                                              uniform_grid_[nearby_cell_id].bbox_max_.y()),
                                          max(uniform_grid_[update_cell_id].bbox_max_.z(),
                                              uniform_grid_[nearby_cell_id].bbox_max_.z()));

          std::vector<Position> path, reverse_path;
          double cost = getAStarCostBBox(uniform_grid_[update_cell_id].centers_free_[i],
                                         uniform_grid_[nearby_cell_id].centers_free_[k],
                                         local_astar_bbox_min, local_astar_bbox_max, path);
          // reverse path
          for (int i = path.size() - 1; i >= 0; i--) {
            reverse_path.push_back(path[i]);
          }

          node->addNeighborWithPath(uniform_grid_[nearby_cell_id].id_ * 10 + k, cost,
                                    ConnectivityEdge::TYPE::FREE, path);
          node_neighbor->addNeighborWithPath(uniform_grid_[update_cell_id].id_ * 10 + i, cost,
                                             ConnectivityEdge::TYPE::FREE, reverse_path);
        }
      }
    }

    // current cell's centers_unknown_ can be connected with nearby cells' centers_unknown_
    // (UNKNOWN)
    for (int i = 0; i < uniform_grid_[update_cell_id].centers_unknown_.size(); i++) {
      ConnectivityNode::Ptr node =
          connectivity_graph_->getNode(uniform_grid_[update_cell_id].id_ * 10 +
                                       uniform_grid_[update_cell_id].centers_free_.size() + i);
      CHECK_NOTNULL(node.get());

      // Add nearby cells' centers_unknown as neighbors
      for (int nearby_cell_id : uniform_grid_[update_cell_id].nearby_cells_ids_) {
        for (int k = 0; k < uniform_grid_[nearby_cell_id].centers_unknown_.size(); k++) {
          ConnectivityNode::Ptr node_neighbor =
              connectivity_graph_->getNode(uniform_grid_[nearby_cell_id].id_ * 10 +
                                           uniform_grid_[nearby_cell_id].centers_free_.size() + k);
          CHECK_NOTNULL(node_neighbor.get());

          Position local_astar_bbox_min, local_astar_bbox_max;
          local_astar_bbox_min = Position(min(uniform_grid_[update_cell_id].bbox_min_.x(),
                                              uniform_grid_[nearby_cell_id].bbox_min_.x()),
                                          min(uniform_grid_[update_cell_id].bbox_min_.y(),
                                              uniform_grid_[nearby_cell_id].bbox_min_.y()),
                                          min(uniform_grid_[update_cell_id].bbox_min_.z(),
                                              uniform_grid_[nearby_cell_id].bbox_min_.z()));
          local_astar_bbox_max = Position(max(uniform_grid_[update_cell_id].bbox_max_.x(),
                                              uniform_grid_[nearby_cell_id].bbox_max_.x()),
                                          max(uniform_grid_[update_cell_id].bbox_max_.y(),
                                              uniform_grid_[nearby_cell_id].bbox_max_.y()),
                                          max(uniform_grid_[update_cell_id].bbox_max_.z(),
                                              uniform_grid_[nearby_cell_id].bbox_max_.z()));

          std::vector<Position> path, reverse_path;
          double cost =
              getAStarCostBBoxUnknownOnly(uniform_grid_[update_cell_id].centers_unknown_[i],
                                          uniform_grid_[nearby_cell_id].centers_unknown_[k],
                                          local_astar_bbox_min, local_astar_bbox_max, path);
          for (int i = path.size() - 1; i >= 0; i--) {
            reverse_path.push_back(path[i]);
          }

          node->addNeighborWithPath(uniform_grid_[nearby_cell_id].id_ * 10 +
                                        uniform_grid_[nearby_cell_id].centers_free_.size() + k,
                                    cost, ConnectivityEdge::TYPE::UNKNOWN, path);
          node_neighbor->addNeighborWithPath(
              uniform_grid_[update_cell_id].id_ * 10 +
                  uniform_grid_[update_cell_id].centers_free_.size() + i,
              cost, ConnectivityEdge::TYPE::UNKNOWN, reverse_path);
        }
      }
    }

    // TODO: if current cells unknown is not connected with any nearby cells' unknown or current
    // cell's free, then it is a isolated cell, try to connect it with nearby cells' free
  }

  std::set<int> disconnected_nodes;
  connectivity_graph_->findDisconnectedNodes(disconnected_nodes);
  if (false) {
    std::cout << "disconnected_nodes: ";
    for (int node_id : disconnected_nodes) {
      std::cout << node_id << " ";
    }
    std::cout << std::endl;
  }

  std::map<int, std::unordered_set<int>> disconnected_cells_ids_centers_ids;
  for (int node_id : disconnected_nodes) {
    int cell_id = node_id / 10;
    int unknown_center_id = node_id % 10 - uniform_grid_[cell_id].centers_free_.size();
    disconnected_cells_ids_centers_ids[cell_id].insert(unknown_center_id);
  }

  // iterate through disconnected_cells_ids_centers_ids
  for (auto it = disconnected_cells_ids_centers_ids.begin();
       it != disconnected_cells_ids_centers_ids.end(); ++it) {
    int cell_id = it->first;
    std::unordered_set<int> centers_ids = it->second;

    if (disconnected_cells_ids_centers_ids.size() == uniform_grid_.size()) {
      ROS_WARN_THROTTLE(1, "[UniformGrid] All cells are disconnected, no need to update");
      break;
    }

    // remove disconnected centers from centers_unknown_active_
    // remove holes in the map
    uniform_grid_[cell_id].centers_unknown_active_.clear();
    for (int i = 0; i < uniform_grid_[cell_id].centers_unknown_.size(); i++) {
      if (centers_ids.find(i) == centers_ids.end()) {
        uniform_grid_[cell_id].centers_unknown_active_.push_back(
            uniform_grid_[cell_id].centers_unknown_[i]);
        uniform_grid_[cell_id].centers_unknown_active_idx_.push_back(i);
      }
    }
  }

  connectivity_graph_time_ = (ros::Time::now() - t1).toSec();
}

void UniformGrid::updateGridsFrontierInfo(const Position &update_bbox_min,
                                          const Position &update_bbox_max) {
  // Get update bounding box
  Position bbox_min = update_bbox_min;
  Position bbox_max = update_bbox_max;

  // Get the cells that have overlaps the update bounding box
  std::vector<int> update_cell_ids;
  getGridCellIdsInBBox(bbox_min, bbox_max, update_cell_ids);

  for (int cell_id : cell_ids_need_update_frontier_) {
    if (std::find(update_cell_ids.begin(), update_cell_ids.end(), cell_id) ==
        update_cell_ids.end()) {
      update_cell_ids.push_back(cell_id);
    }
  }

  for (int update_cell_id : update_cell_ids) {
    Position cell_min, cell_max;
    cell_min = uniform_grid_[update_cell_id].bbox_min_;
    cell_max = uniform_grid_[update_cell_id].bbox_max_;

    if (false) {
      // Recompute center if frontier num > 0, center is the average of all frontier viewpoints
      if (uniform_grid_[update_cell_id].frontier_num_ > 0) {
        Position frontier_center = Position::Zero();
        for (const Position &frontier_viewpoint :
             uniform_grid_[update_cell_id].frontier_viewpoints_) {
          frontier_center = frontier_center + frontier_viewpoint;
        }
        uniform_grid_[update_cell_id].center_ = rectifyCellCenter(
            update_cell_id, frontier_center / uniform_grid_[update_cell_id].frontier_num_, cell_min,
            cell_max);

        // print frontier viewpoints
        if (config_.verbose_) {
          std::string frontier_viewpoint_string;
          for (const Position &frontier_viewpoint :
               uniform_grid_[update_cell_id].frontier_viewpoints_) {
            frontier_viewpoint_string += "(" + std::to_string(frontier_viewpoint.x()) + ", " +
                                         std::to_string(frontier_viewpoint.y()) + ", " +
                                         std::to_string(frontier_viewpoint.z()) + ") ";
          }
          ROS_INFO("[UniformGrid] Cell %d has %d frontiers, center: (%f, %f, %f), viewpoints: %s",
                   update_cell_id, uniform_grid_[update_cell_id].frontier_num_,
                   uniform_grid_[update_cell_id].center_.x(),
                   uniform_grid_[update_cell_id].center_.y(),
                   uniform_grid_[update_cell_id].center_.z(), frontier_viewpoint_string.c_str());
        }
      }
    }

    /*******************************************************************************
     * Cluster frontiers into corresponding free subspaces
     ********************************************************************************/
    // Clear previous active free subspaces
    uniform_grid_[update_cell_id].centers_free_active_.clear();
    uniform_grid_[update_cell_id].centers_free_active_idx_.clear();
    uniform_grid_[update_cell_id].frontier_ids_mc_.clear();
    uniform_grid_[update_cell_id].frontier_viewpoints_mc_.clear();
    uniform_grid_[update_cell_id].frontier_yaws_mc_.clear();

    // centers_free_ including active free subspaces and inactive free subspaces (no viewpoints)
    // Only unknown subspaces and active free subspaces are considered in CP
    // As they are needed to be traversed by the robot during exploration
    if (uniform_grid_[update_cell_id].frontier_num_ == 0) {
      // No frontier in this cell, all free centers are removed from global CP
      uniform_grid_[update_cell_id].centers_free_active_.clear();
      uniform_grid_[update_cell_id].centers_free_active_idx_.clear();
      // ROS_INFO("[UniformGrid] Cell %d has no frontiers for clustering", update_cell_id);
    } else {
      // Has frontiers in this cell, cluster frontiers into corresponding free subspaces
      if (uniform_grid_[update_cell_id].centers_free_.size() == 0) {
        ROS_ERROR("[UniformGrid] Cell %d has %d frontiers, but no free subspace", update_cell_id,
                  uniform_grid_[update_cell_id].frontier_num_);
        // CHECK(false) << "Cell " << update_cell_id << " has "
        //              << uniform_grid_[update_cell_id].frontier_num_
        //              << " frontiers, but no free subspace";

        Position center_rect = Eigen::Vector3d::Zero();
        for (const Position &frontier_viewpoint :
             uniform_grid_[update_cell_id].frontier_viewpoints_) {
          center_rect = center_rect + frontier_viewpoint;
        }
        center_rect = center_rect / uniform_grid_[update_cell_id].frontier_num_;
        uniform_grid_[update_cell_id].centers_free_active_.push_back(center_rect);
        uniform_grid_[update_cell_id].centers_free_active_idx_.push_back(-1); // Hack, to be fixed

        uniform_grid_[update_cell_id].frontier_ids_mc_.push_back(
            uniform_grid_[update_cell_id].frontier_ids_);
        uniform_grid_[update_cell_id].frontier_viewpoints_mc_.push_back(
            uniform_grid_[update_cell_id].frontier_viewpoints_);
        uniform_grid_[update_cell_id].frontier_yaws_mc_.push_back(
            uniform_grid_[update_cell_id].frontier_yaws_);

      } else if (uniform_grid_[update_cell_id].centers_free_.size() == 1) {
        // Only one free subspace, all frontier viewpoints should be in this subspace
        Position center_rect = Eigen::Vector3d::Zero();
        for (const Position &frontier_viewpoint :
             uniform_grid_[update_cell_id].frontier_viewpoints_) {
          center_rect = center_rect + frontier_viewpoint;
        }
        center_rect = center_rect / uniform_grid_[update_cell_id].frontier_num_;
        // uniform_grid_[update_cell_id].centers_free_active_.push_back(
        //     rectifyCellCenter(update_cell_id, center_rect, cell_min, cell_max));

        const std::vector<std::pair<int, int>> &free_unknown_states_and_centers_idx =
            ccl_free_unknown_states_and_centers_idx_[update_cell_id];
        int ccl_voxels_addr_idx = 0;
        bool found = false;
        for (const std::pair<int, int> &free_unknown_state_and_center_idx :
             free_unknown_states_and_centers_idx) {
          // cout << "Cell id: " << update_cell_id << ", free_unknown_state_and_center_idx: ("
          //      << free_unknown_state_and_center_idx.first << ", "
          //      << free_unknown_state_and_center_idx.second << ")" << endl;

          if (free_unknown_state_and_center_idx.first == 0 &&
              free_unknown_state_and_center_idx.second == 0) {
            found = true;
            break;
          }

          ccl_voxels_addr_idx++;
        }

        CHECK(found) << "Cell " << update_cell_id << " ccl_voxels_addr_idx incorrect";

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        ccl_voxels_color_[update_cell_id][ccl_voxels_addr_idx] =
            Eigen::Vector3d(252.0 / 255.0, 248.0 / 255.0, 123.0 / 255.0);
        const std::unordered_set<int> &voxel_addr_set =
            ccl_voxels_addr_[update_cell_id][ccl_voxels_addr_idx];
        for (const int &voxel_addr : voxel_addr_set) {
          Position voxel_pos = map_server_->getTSDF()->addressToPosition(voxel_addr);
          cloud->push_back(pcl::PointXYZ(voxel_pos.x(), voxel_pos.y(), voxel_pos.z()));
        }
        kdtree.setInputCloud(cloud);

        pcl::PointXYZ searchPoint(center_rect.x(), center_rect.y(), center_rect.z());

        // K nearest neighbor search
        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
          pcl::PointXYZ nearest_point = cloud->points[pointIdxNKNSearch[0]];
          center_rect = Position(nearest_point.x, nearest_point.y, nearest_point.z);
        } else {
          ROS_ERROR("[UniformGrid] K nearest neighbor search failed!");
          CHECK(false) << "K nearest neighbor search failed!";
        }

        uniform_grid_[update_cell_id].centers_free_active_.push_back(center_rect);
        uniform_grid_[update_cell_id].centers_free_active_idx_.push_back(0);

        uniform_grid_[update_cell_id].frontier_ids_mc_.push_back(
            uniform_grid_[update_cell_id].frontier_ids_);
        uniform_grid_[update_cell_id].frontier_viewpoints_mc_.push_back(
            uniform_grid_[update_cell_id].frontier_viewpoints_);
        uniform_grid_[update_cell_id].frontier_yaws_mc_.push_back(
            uniform_grid_[update_cell_id].frontier_yaws_);

        // print frontier id
        // std::string frontier_id_string;
        // for (int frontier_id : uniform_grid_[update_cell_id].frontier_ids_) {
        //   frontier_id_string += std::to_string(frontier_id) + " ";
        // }
        // ROS_INFO("[UniformGrid] Cell %d has %lu frontiers, center: (%f, %f, %f), ids: %s",
        //          update_cell_id, uniform_grid_[update_cell_id].frontier_ids_.size(),
        //          center_rect.x(), center_rect.y(), center_rect.z(),
        //          frontier_id_string.c_str());
      } else {
        // Multiple free subspaces
        // Iterate through all free subregions
        // size = frontier_viewpoints.size() * centers_free.size()
        vector<vector<double>> cost_mat_frontier_vps_centers_free(
            uniform_grid_[update_cell_id].frontier_viewpoints_.size(),
            vector<double>(uniform_grid_[update_cell_id].centers_free_.size(), 1000.0));

        for (int frontier_idx = 0;
             frontier_idx < uniform_grid_[update_cell_id].frontier_viewpoints_.size();
             frontier_idx++) {
          Position frontier_viewpoint =
              uniform_grid_[update_cell_id].frontier_viewpoints_[frontier_idx];

          int center_free_idx = 0;
          for (const Position &center_free : uniform_grid_[update_cell_id].centers_free_) {
            // computeCostUnknownBBox
            double cost = getAStarCostBBox(frontier_viewpoint, center_free,
                                           uniform_grid_[update_cell_id].bbox_min_,
                                           uniform_grid_[update_cell_id].bbox_max_);
            cost_mat_frontier_vps_centers_free[frontier_idx][center_free_idx] = cost;
            center_free_idx++;
          }
        }

        // For each frontier, find the minimum cost center
        std::vector<int> unclassified_frontier_idx;
        std::vector<std::vector<int>> frontier_idx_mc;
        // For each free center, save the frontier id that are connected to it with minimum cost
        frontier_idx_mc.resize(uniform_grid_[update_cell_id].centers_free_.size());
        for (int frontier_idx = 0;
             frontier_idx < uniform_grid_[update_cell_id].frontier_viewpoints_.size();
             frontier_idx++) {
          std::vector<double>::iterator min_cost_center_free_it =
              std::min_element(cost_mat_frontier_vps_centers_free[frontier_idx].begin(),
                               cost_mat_frontier_vps_centers_free[frontier_idx].end());
          if (min_cost_center_free_it == cost_mat_frontier_vps_centers_free[frontier_idx].end())
            CHECK(false) << "min_element error";
          double min_cost_center_free = *min_cost_center_free_it;
          int min_cost_center_free_idx = std::distance(
              cost_mat_frontier_vps_centers_free[frontier_idx].begin(), min_cost_center_free_it);

          if (min_cost_center_free > 499.0) {
            // No free center is connected to this frontier viewpoint
            // This frontier will be treated as a unclassified frontier
            unclassified_frontier_idx.push_back(frontier_idx);
            ROS_ERROR("[UniformGrid] Cell %d has %d frontiers, but no free subspace is connected "
                      "to frontier %d",
                      update_cell_id, uniform_grid_[update_cell_id].frontier_num_,
                      uniform_grid_[update_cell_id].frontier_ids_[frontier_idx]);
            // CHECK(false) << "No free center is connected to this frontier viewpoint";

            continue;
          }

          // Add this frontier viewpoint to the corresponding free subspace
          frontier_idx_mc[min_cost_center_free_idx].push_back(frontier_idx);
        }

        // add unclassified frontiers to all free subspaces
        for (std::vector<int> &frontier_idx : frontier_idx_mc) {
          frontier_idx.insert(frontier_idx.end(), unclassified_frontier_idx.begin(),
                              unclassified_frontier_idx.end());
        }

        for (int centers_free_idx = 0;
             centers_free_idx < (int)uniform_grid_[update_cell_id].centers_free_.size();
             centers_free_idx++) {
          if (frontier_idx_mc[centers_free_idx].size() == 0) {
            // No frontier viewpoint in this free subspace
            continue;
          }

          Position center_rect = Eigen::Vector3d::Zero();
          for (int frontier_idx : frontier_idx_mc[centers_free_idx]) {
            center_rect =
                center_rect + uniform_grid_[update_cell_id].frontier_viewpoints_[frontier_idx];
          }
          center_rect = center_rect / frontier_idx_mc[centers_free_idx].size();
          // uniform_grid_[update_cell_id].centers_free_active_.push_back(
          //     rectifyCellCenter(update_cell_id, center_rect, cell_min, cell_max));

          {
            const std::vector<std::pair<int, int>> &free_unknown_states_and_centers_idx =
                ccl_free_unknown_states_and_centers_idx_[update_cell_id];
            int ccl_voxels_addr_idx = 0;
            bool found = false;
            for (const std::pair<int, int> &free_unknown_state_and_center_idx :
                 free_unknown_states_and_centers_idx) {
              // cout << "Cell id: " << update_cell_id << ", free_unknown_state_and_center_idx: ("
              //      << free_unknown_state_and_center_idx.first << ", "
              //      << free_unknown_state_and_center_idx.second << ")" << endl;

              if (free_unknown_state_and_center_idx.first == 0 &&
                  free_unknown_state_and_center_idx.second == centers_free_idx) {
                found = true;
                break;
              }

              ccl_voxels_addr_idx++;
            }

            CHECK(found) << "Cell " << update_cell_id << " ccl_voxels_addr_idx incorrect";

            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

            const std::unordered_set<int> &voxel_addr_set =
                ccl_voxels_addr_[update_cell_id][ccl_voxels_addr_idx];
            ccl_voxels_color_[update_cell_id][ccl_voxels_addr_idx] =
                Eigen::Vector3d(252.0 / 255.0, 248.0 / 255.0, 123.0 / 255.0);
            for (const int &voxel_addr : voxel_addr_set) {
              Position voxel_pos = map_server_->getTSDF()->addressToPosition(voxel_addr);
              cloud->push_back(pcl::PointXYZ(voxel_pos.x(), voxel_pos.y(), voxel_pos.z()));
            }
            kdtree.setInputCloud(cloud);

            pcl::PointXYZ searchPoint(center_rect.x(), center_rect.y(), center_rect.z());

            // K nearest neighbor search
            int K = 1;
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);

            if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) >
                0) {
              pcl::PointXYZ nearest_point = cloud->points[pointIdxNKNSearch[0]];
              center_rect = Position(nearest_point.x, nearest_point.y, nearest_point.z);
            } else {
              ROS_ERROR("[UniformGrid] K nearest neighbor search failed!");
              CHECK(false) << "K nearest neighbor search failed!";
            }

            uniform_grid_[update_cell_id].centers_free_active_.push_back(center_rect);
            uniform_grid_[update_cell_id].centers_free_active_idx_.push_back(centers_free_idx);
          }

          std::vector<int> frontier_ids_mc;
          std::vector<Position> frontier_viewpoints_mc;
          std::vector<double> frontier_yaws_mc;
          for (int frontier_idx : frontier_idx_mc[centers_free_idx]) {
            frontier_ids_mc.push_back(uniform_grid_[update_cell_id].frontier_ids_[frontier_idx]);
            frontier_viewpoints_mc.push_back(
                uniform_grid_[update_cell_id].frontier_viewpoints_[frontier_idx]);
            frontier_yaws_mc.push_back(uniform_grid_[update_cell_id].frontier_yaws_[frontier_idx]);
          }

          uniform_grid_[update_cell_id].frontier_ids_mc_.push_back(frontier_ids_mc);
          uniform_grid_[update_cell_id].frontier_viewpoints_mc_.push_back(frontier_viewpoints_mc);
          uniform_grid_[update_cell_id].frontier_yaws_mc_.push_back(frontier_yaws_mc);
        }

        // Check if all frontiers are classified
        int num_frontiers = uniform_grid_[update_cell_id].frontier_viewpoints_.size();
        int num_classified_frontiers = 0;
        for (int centers_free_idx = 0;
             centers_free_idx < (int)uniform_grid_[update_cell_id].centers_free_.size();
             centers_free_idx++) {
          num_classified_frontiers += frontier_idx_mc[centers_free_idx].size();
        }

        // Print information about multiple free subspaces
        if (config_.verbose_) {
          ROS_INFO("[UniformGrid] Cell %d has %d frontiers, %lu free subspaces, %d classified "
                   "frontiers, %d unclassified frontiers",
                   update_cell_id, num_frontiers,
                   uniform_grid_[update_cell_id].centers_free_.size(), num_classified_frontiers,
                   num_frontiers - num_classified_frontiers);
          for (int centers_free_idx = 0;
               centers_free_idx < (int)uniform_grid_[update_cell_id].centers_free_active_.size();
               centers_free_idx++) {
            string frontier_id_string;
            for (int frontier_id :
                 uniform_grid_[update_cell_id].frontier_ids_mc_[centers_free_idx]) {
              frontier_id_string += std::to_string(frontier_id) + " ";
            }

            ROS_INFO("[UniformGrid] Cell %d has %lu frontiers in free subspace %d, center: (%f, "
                     "%f, %f), ids: %s",
                     update_cell_id, frontier_idx_mc[centers_free_idx].size(), centers_free_idx,
                     uniform_grid_[update_cell_id].centers_free_active_[centers_free_idx].x(),
                     uniform_grid_[update_cell_id].centers_free_active_[centers_free_idx].y(),
                     uniform_grid_[update_cell_id].centers_free_active_[centers_free_idx].z(),
                     frontier_id_string.c_str());
          }
        }
      }
    }
  }
}

Position UniformGrid::rectifyCellCenter(const int &cell_id, const Position &center,
                                        const Position &bbox_min, const Position &bbox_max) {
  // Recitify cell center to avoid obstacles and unknown holes
  Position center_rectified = center;

  // TODO: Check if the cell center is unknown, move it to the nearest known cell center

  // Check if the cell center is in an obstacle
  double esdf_dist;
  Eigen::Vector3d esdf_grad;
  map_server_->getESDF()->getDistanceAndGradient(center, esdf_dist, esdf_grad);

  // Push center along the gradient direction
  if (esdf_dist < 0.7) {
    center_rectified = center + (0.7 - esdf_dist) * esdf_grad.normalized();
  }

  if (abs(esdf_dist) < 1e-4 && esdf_grad.norm() < 1e-4 &&
      map_server_->getOccupancy(center) == voxel_mapping::OccupancyType::OCCUPIED) {
    // ROS_INFO("[UniformGrid] Cell %d center is inside an object, esdf_dist: %f, grad: (%f,
    // %f,%f)",
    //          cell_id, esdf_dist, esdf_grad.x(), esdf_grad.y(), esdf_grad.z());
    // Push center along the gradient direction
    // center_rectified = center + Eigen::Vector3d(0.7, 0.0, 0.0);

    // Uniform sample on a unit circle
    double radius = 0.5;
    int N_sample = 8;
    std::vector<Eigen::Vector3d> samples;
    for (int i = 0; i < N_sample; i++) {
      double theta = 2 * M_PI * i / N_sample;
      Eigen::Vector3d sample = Eigen::Vector3d(radius * cos(theta), radius * sin(theta), 0.0);
      samples.push_back(sample);
    }

    // Find the nearest sample that is not in an obstacle
    Eigen::Vector3d nearest_sample;
    for (Eigen::Vector3d sample : samples) {
      Position sample_pos = center + sample;
      map_server_->getESDF()->getDistanceAndGradient(sample_pos, esdf_dist, esdf_grad);
      if (esdf_dist > 1e-4 && esdf_grad.norm() > 1e-4) {
        // Found a valid center
        center_rectified = sample_pos + (0.7 - esdf_dist) * esdf_grad.normalized();
      }
    }
  }

  // Bound to bbox
  center_rectified.x() =
      std::max(bbox_min.x() + 1e-4, std::min(bbox_max.x() - 1e-4, center_rectified.x()));
  center_rectified.y() =
      std::max(bbox_min.y() + 1e-4, std::min(bbox_max.y() - 1e-4, center_rectified.y()));
  center_rectified.z() =
      std::max(bbox_min.z() + 1e-4, std::min(bbox_max.z() - 1e-4, center_rectified.z()));

  return center_rectified;
}

void UniformGrid::getCCLCenters(const int &cell_id, std::vector<Position> &centers_free,
                                std::vector<Position> &centers_unknown) {
  // https://en.wikipedia.org/wiki/Connected-component_labeling
  // ROS_INFO("[UniformGrid] Get CCL centers for cell %d", cell_id);

  ros::Time start_time = ros::Time::now();

  centers_free.clear();
  centers_unknown.clear();
  ccl_voxels_addr_[cell_id].clear();
  ccl_voxels_color_[cell_id].clear();

  // double height = 1.0;
  Position bbox_min = uniform_grid_[cell_id].bbox_min_;
  Position bbox_max = uniform_grid_[cell_id].bbox_max_;
  // bbox_min.z() = height;
  // bbox_max.z() = height;

  VoxelIndex bbox_min_idx = map_server_->getTSDF()->positionToIndex(bbox_min);
  VoxelIndex bbox_max_idx = map_server_->getTSDF()->positionToIndex(bbox_max);

  // Get the center of the cell using CCL
  int size_x = bbox_max_idx.x() - bbox_min_idx.x();
  int size_y = bbox_max_idx.y() - bbox_min_idx.y();
  int size_z = bbox_max_idx.z() - bbox_min_idx.z();
  // cout << "bbox_min_idx: " << bbox_min_idx.transpose() << endl;
  // cout << "bbox_max_idx: " << bbox_max_idx.transpose() << endl;
  // cout << "size_x: " << size_x << endl;
  // cout << "size_y: " << size_y << endl;
  std::vector<int> labels(size_x * size_y * size_z, 0);
  int step = config_.ccl_step_;
  int freeSpaces = 0;
  int label = 1;
  // int index_z = bbox_min_idx.z();
  int min_unknown_num = size_x * size_y * size_z / pow(step, 3) * 0.1;
  // cout << "min_unknown_num: " << min_unknown_num << endl;

  int free_unknown_state = -1; // 0: free, 1: unknown
  std::vector<std::pair<int, int>> &free_unknown_states_and_centers_idx =
      ccl_free_unknown_states_and_centers_idx_[cell_id];
  free_unknown_states_and_centers_idx.clear();

  auto isFree = [&](const VoxelIndex &idx) {
    free_unknown_state = 0;
    // return map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::FREE;

    // Change to TSDF value
    return map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::FREE;
  };

  auto isFreeInflated = [&](const VoxelIndex &idx) {
    free_unknown_state = 0;
    // return map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::FREE;

    // Change to TSDF value
    return map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::FREE &&
           map_server_->getESDF()->getVoxel(idx).value > 0.5;
  };

  auto isUnknown = [&](const VoxelIndex &idx) {
    free_unknown_state = 1;
    return map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::UNKNOWN;
  };

  // ROS_INFO("[UniformGrid] Cell %d, starting CCL", cell_id);

  // CCL one pass
  for (int z = 0; z < size_z; z = z + step) {
    for (int y = 0; y < size_y; y = y + step) {
      for (int x = 0; x < size_x; x = x + step) {
        free_unknown_state = -1;
        int index = z * size_x * size_y + y * size_x + x;

        // Unlabelled voxel will be checked
        VoxelIndex start_idx =
            VoxelIndex(x + bbox_min_idx.x(), y + bbox_min_idx.y(), z + bbox_min_idx.z());
        if (labels[index] == 0 && (isFreeInflated(start_idx) || isUnknown(start_idx))) {
          std::stack<int> stack;
          stack.push(index);
          labels[index] = label;
          // std::cout << "Current xyz: " << x << " " << y << ", index: " << index << std::endl;

          int num_voxels = 0;
          double X_avg = 0.0;
          double Y_avg = 0.0;
          double Z_avg = 0.0;

          std::unordered_set<int> voxel_addrs;
          Eigen::Vector3d voxel_color = Eigen::Vector3d::Random();

          while (!stack.empty()) {
            num_voxels++;
            // print stack
            // std::cout << "Stack size: " << stack.size() << std::endl;

            int currIndex = stack.top();
            // std::cout << "Current index: " << currIndex << std::endl;

            int currZ = currIndex / (size_x * size_y);
            int currY = (currIndex - currZ * size_x * size_y) / size_x;
            int currX = currIndex - currZ * size_x * size_y - currY * size_x;
            VoxelIndex curr_voxel_idx = VoxelIndex(
                currX + bbox_min_idx.x(), currY + bbox_min_idx.y(), currZ + bbox_min_idx.z());
            int curr_voxel_addr = map_server_->getTSDF()->indexToAddress(curr_voxel_idx);
            voxel_addrs.insert(curr_voxel_addr);

            X_avg += currX;
            Y_avg += currY;
            Z_avg += currZ;

            stack.pop();
            for (int dz = -step; dz <= step; dz += step) {
              for (int dy = -step; dy <= step; dy += step) {
                for (int dx = -step; dx <= step; dx += step) {
                  if (dx == 0 && dy == 0 && dz == 0)
                    continue;
                  int nx = currX + dx;
                  int ny = currY + dy;
                  int nz = currZ + dz;

                  // out of bounds
                  if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y || nz < 0 || nz >= size_z) {
                    continue;
                  }

                  int neighborIndex = nz * size_x * size_y + ny * size_x + nx;

                  if (labels[neighborIndex] == 0 && free_unknown_state == 0 &&
                      isFreeInflated(VoxelIndex(nx + bbox_min_idx.x(), ny + bbox_min_idx.y(),
                                                nz + bbox_min_idx.z()))) {
                    // Check middle voxels between curr voxel and neighbor voxel
                    bool isMiddleVoxelFree = true;
                    for (int ddz = 0; ddz < step; ddz++) {
                      for (int ddy = 0; ddy < step; ddy++) {
                        for (int ddx = 0; ddx < step; ddx++) {
                          if (ddx == 0 && ddy == 0 && ddz == 0)
                            continue;
                          int mx = currX + ddx;
                          int my = currY + ddy;
                          int mz = currZ + ddz;
                          if (!isFreeInflated(VoxelIndex(mx + bbox_min_idx.x(),
                                                         my + bbox_min_idx.y(),
                                                         mz + bbox_min_idx.z()))) {
                            isMiddleVoxelFree = false;
                            continue;
                          }
                        }
                      }
                    }

                    if (!isMiddleVoxelFree) {
                      continue;
                    }

                    stack.push(neighborIndex);
                    labels[neighborIndex] = label;
                  }

                  if (labels[neighborIndex] == 0 && free_unknown_state == 1 &&
                      isUnknown(VoxelIndex(nx + bbox_min_idx.x(), ny + bbox_min_idx.y(),
                                           nz + bbox_min_idx.z()))) {
                    bool isMiddleVoxelUnknown = true;
                    for (int ddz = 0; ddz < step; ddz++) {
                      for (int ddy = 0; ddy < step; ddy++) {
                        for (int ddx = 0; ddx < step; ddx++) {
                          if (ddx == 0 && ddy == 0 && ddz == 0)
                            continue;
                          int mx = currX + ddx;
                          int my = currY + ddy;
                          int mz = currZ + ddz;
                          if (!isUnknown(VoxelIndex(mx + bbox_min_idx.x(), my + bbox_min_idx.y(),
                                                    mz + bbox_min_idx.z()))) {
                            isMiddleVoxelUnknown = false;
                            continue;
                          }
                        }
                      }
                    }

                    if (!isMiddleVoxelUnknown) {
                      continue;
                    }

                    stack.push(neighborIndex);
                    labels[neighborIndex] = label;
                  }
                }
              }
            }
          }

          X_avg /= (double)num_voxels;
          Y_avg /= (double)num_voxels;
          Z_avg /= (double)num_voxels;

          // cout << "num_voxels: " << num_voxels << endl;
          // cout << "X_avg: " << X_avg * map_server_->getResolution() + bbox_min.x() << endl;
          // cout << "Y_avg: " << Y_avg * map_server_->getResolution() + bbox_min.y() << endl;

          // ROS_INFO("[UniformGrid] CCL center: (%.2f, %.2f, %.2f)",
          //          X_avg * map_server_->getResolution() + bbox_min.x(),
          //          Y_avg * map_server_->getResolution() + bbox_min.y(), height);

          // Remove small free subspaces (treated as noise)
          if (num_voxels < 10)
            continue;

          if (free_unknown_state == 1) {
            if (num_voxels < min_unknown_num)
              continue;
          }

          ccl_voxels_addr_[cell_id].push_back(voxel_addrs);
          Eigen::Vector3d voxel_color_free =
              Eigen::Vector3d(227.0 / 255.0, 187.0 / 255.0, 247.0 / 255.0);
          Eigen::Vector3d voxel_color_unknown =
              Eigen::Vector3d(230.0 / 255.0, 230.0 / 255.0, 230.0 / 255.0);

          // ROS_INFO(
          //     "[UniformGrid] Cell %d, CCL center: (%.2f, %.2f, %.2f), num_voxels: %d, state: %d",
          //     cell_id, X_avg * map_server_->getResolution() + bbox_min.x(),
          //     Y_avg * map_server_->getResolution() + bbox_min.y(),
          //     Z_avg * map_server_->getResolution() + bbox_min.z(), num_voxels,
          //     free_unknown_state);
          if (free_unknown_state == 0) {
            free_unknown_states_and_centers_idx.push_back(
                std::make_pair(free_unknown_state, centers_free.size()));
            centers_free.push_back(Position(X_avg * map_server_->getResolution() + bbox_min.x(),
                                            Y_avg * map_server_->getResolution() + bbox_min.y(),
                                            Z_avg * map_server_->getResolution() + bbox_min.z()));
            ccl_voxels_color_[cell_id].push_back(voxel_color_free);
          } else if (free_unknown_state == 1) {
            free_unknown_states_and_centers_idx.push_back(
                std::make_pair(free_unknown_state, centers_unknown.size()));
            centers_unknown.push_back(
                Position(X_avg * map_server_->getResolution() + bbox_min.x(),
                         Y_avg * map_server_->getResolution() + bbox_min.y(),
                         Z_avg * map_server_->getResolution() + bbox_min.z()));
            ccl_voxels_color_[cell_id].push_back(voxel_color_unknown);
          } else
            CHECK(false) << "free_unknown_state is not set!";

          freeSpaces++;
          label++;
        }
      }
    }
  }

  CHECK_EQ(ccl_voxels_addr_[cell_id].size(), free_unknown_states_and_centers_idx.size())
      << "CCL centers number mismatch!" << ccl_voxels_addr_[cell_id].size() << " vs "
      << free_unknown_states_and_centers_idx.size();

  // ROS_INFO("[UniformGrid] Cell %d Found %d centers_free, %d centers_unknown", cell_id,
  //          centers_free.size(), centers_unknown.size());

  int ccl_voxels_idx = 0;
  for (const std::pair<int, int> &free_unknown_state_and_center_idx :
       free_unknown_states_and_centers_idx) {

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    const std::unordered_set<int> &voxel_addr_set = ccl_voxels_addr_[cell_id][ccl_voxels_idx];
    for (const int &voxel_addr : voxel_addr_set) {
      Position voxel_pos = map_server_->getTSDF()->addressToPosition(voxel_addr);
      cloud->push_back(pcl::PointXYZ(voxel_pos.x(), voxel_pos.y(), voxel_pos.z()));
    }
    kdtree.setInputCloud(cloud);
    ccl_voxels_idx++;

    Position center = free_unknown_state_and_center_idx.first == 0
                          ? centers_free[free_unknown_state_and_center_idx.second]
                          : centers_unknown[free_unknown_state_and_center_idx.second];
    pcl::PointXYZ searchPoint(center.x(), center.y(), center.z());
    // ROS_INFO("[UniformGrid] Cell %d CCL center before: (%.2f, %.2f, %.2f)", cell_id, center.x(),
    //          center.y(), center.z());

    // K nearest neighbor search
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      pcl::PointXYZ nearest_point = cloud->points[pointIdxNKNSearch[0]];
      center = Position(nearest_point.x, nearest_point.y, nearest_point.z);
    } else {
      ROS_ERROR("[UniformGrid] K nearest neighbor search failed!");
    }

    if (free_unknown_state_and_center_idx.first == 0) {
      centers_free[free_unknown_state_and_center_idx.second] = center;
    } else if (free_unknown_state_and_center_idx.first == 1) {
      centers_unknown[free_unknown_state_and_center_idx.second] = center;
    } else {
      CHECK(false) << "free_unknown_state is not set!";
    }

    // ROS_INFO("[UniformGrid] Cell %d CCL center after: (%.2f, %.2f, %.2f)", cell_id, center.x(),
    //          center.y(), center.z());
  }

  // ROS_INFO("[UniformGrid] Cell id: %d Found %d centers_free", cell_id, centers_free.size());
  // ROS_INFO("[UniformGrid] Cell id: %d Found %d centers_unknown", cell_id,
  // centers_unknown.size());

  // Too many subspaces may cause visualization error
  CHECK_LE(freeSpaces, 10) << "Too many spaces in a cell!";
  // ROS_INFO("[UniformGrid] Found %d free spaces for cell %d", freeSpaces, cell_id);

  ros::Time end_time = ros::Time::now();
  // ROS_INFO("[UniformGrid] Cell id: %d, CCL time: %f ms", cell_id,
  //          (end_time - start_time).toSec() * 1000.0);
}

void UniformGrid::getCCLCenters2D(const int &cell_id, std::vector<Position> &centers_free,
                                  std::vector<Position> &centers_unknown) {
  // ROS_INFO("[UniformGrid] Get CCL centers for cell %d", cell_id);

  centers_free.clear();
  centers_unknown.clear();
  ccl_voxels_addr_[cell_id].clear();
  ccl_voxels_color_[cell_id].clear();

  double height = 1.0;
  Position bbox_min = uniform_grid_[cell_id].bbox_min_;
  Position bbox_max = uniform_grid_[cell_id].bbox_max_;
  bbox_min.z() = height;
  bbox_max.z() = height;

  VoxelIndex bbox_min_idx = map_server_->getTSDF()->positionToIndex(bbox_min);
  VoxelIndex bbox_max_idx = map_server_->getTSDF()->positionToIndex(bbox_max);

  // Get the center of the cell using CCL
  int size_x = bbox_max_idx.x() - bbox_min_idx.x();
  int size_y = bbox_max_idx.y() - bbox_min_idx.y();
  // cout << "bbox_min_idx: " << bbox_min_idx.transpose() << endl;
  // cout << "bbox_max_idx: " << bbox_max_idx.transpose() << endl;
  // cout << "size_x: " << size_x << endl;
  // cout << "size_y: " << size_y << endl;
  std::vector<int> labels(size_x * size_y, 0);
  int step = config_.ccl_step_;
  int freeSpaces = 0;
  int label = 1;
  int index_z = bbox_min_idx.z();

  int free_unknown_state = -1; // 0: free, 1: unknown
  std::vector<std::pair<int, int>> &free_unknown_states_and_centers_idx =
      ccl_free_unknown_states_and_centers_idx_[cell_id];
  free_unknown_states_and_centers_idx.clear();

  auto isFree = [&](const VoxelIndex &idx) {
    free_unknown_state = 0;
    return map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::FREE;
  };

  auto isFreeInflated = [&](const VoxelIndex &idx) {
    free_unknown_state = 0;
    return map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::FREE &&
           map_server_->getESDF()->getVoxel(idx).value > 0.5;
  };

  auto isUnknown = [&](const VoxelIndex &idx) {
    free_unknown_state = 1;
    return map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::UNKNOWN;
  };

  // CCL one pass
  for (int y = 0; y < size_y; y = y + step) {
    for (int x = 0; x < size_x; x = x + step) {
      free_unknown_state = -1;
      int index = y * size_x + x;
      if (labels[index] == 0 &&
          (isFree(VoxelIndex(x + bbox_min_idx.x(), y + bbox_min_idx.y(), index_z)) ||
           isUnknown(VoxelIndex(x + bbox_min_idx.x(), y + bbox_min_idx.y(), index_z)))) {
        std::stack<int> stack;
        stack.push(index);
        labels[index] = label;

        int num_voxels = 0;
        double X_avg = 0.0;
        double Y_avg = 0.0;

        std::unordered_set<int> voxel_addrs;

        while (!stack.empty()) {
          num_voxels++;

          int currIndex = stack.top();

          int currY = currIndex / size_x;
          int currX = currIndex - currY * size_x;
          int curr_voxel_addr = map_server_->getTSDF()->indexToAddress(
              VoxelIndex(currX + bbox_min_idx.x(), currY + bbox_min_idx.y(), index_z));
          voxel_addrs.insert(curr_voxel_addr);

          X_avg += currX;
          Y_avg += currY;

          stack.pop();
          for (int dy = -step; dy <= step; dy += step) {
            for (int dx = -step; dx <= step; dx += step) {
              if (dx == 0 && dy == 0)
                continue;
              int nx = currX + dx;
              int ny = currY + dy;

              // out of bounds
              if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y) {
                continue;
              }

              int neighborIndex = ny * size_x + nx;
              if (labels[neighborIndex] == 0 && free_unknown_state == 0 &&
                  isFree(VoxelIndex(nx + bbox_min_idx.x(), ny + bbox_min_idx.y(), index_z))) {
                bool isMiddleVoxelFree = true;
                for (int ddy = 0; ddy < step; ddy++) {
                  for (int ddx = 0; ddx < step; ddx++) {
                    if (ddx == 0 && ddy == 0)
                      continue;
                    int mx = currX + ddx;
                    int my = currY + ddy;
                    if (!isFreeInflated(
                            VoxelIndex(mx + bbox_min_idx.x(), my + bbox_min_idx.y(), index_z))) {
                      isMiddleVoxelFree = false;
                      continue;
                    }
                  }
                }

                if (!isMiddleVoxelFree) {
                  continue;
                }

                stack.push(neighborIndex);
                labels[neighborIndex] = label;
              }

              if (labels[neighborIndex] == 0 && free_unknown_state == 1 &&
                  isUnknown(VoxelIndex(nx + bbox_min_idx.x(), ny + bbox_min_idx.y(), index_z))) {
                bool isMiddleVoxelUnknown = true;
                for (int ddy = 0; ddy < step; ddy++) {
                  for (int ddx = 0; ddx < step; ddx++) {
                    if (ddx == 0 && ddy == 0)
                      continue;
                    int mx = currX + ddx;
                    int my = currY + ddy;
                    if (!isUnknown(
                            VoxelIndex(mx + bbox_min_idx.x(), my + bbox_min_idx.y(), index_z))) {
                      isMiddleVoxelUnknown = false;
                      continue;
                    }
                  }
                }

                if (!isMiddleVoxelUnknown) {
                  continue;
                }

                stack.push(neighborIndex);
                labels[neighborIndex] = label;
              }
            }
          }
        }

        X_avg /= (double)num_voxels;
        Y_avg /= (double)num_voxels;

        // ROS_INFO("[UniformGrid] CCL center: (%.2f, %.2f, %.2f)",
        //          X_avg * map_server_->getResolution() + bbox_min.x(),
        //          Y_avg * map_server_->getResolution() + bbox_min.y(), height);

        if (num_voxels < 10)
          continue;

        if (free_unknown_state == 1 && num_voxels < 0.1 * size_x * size_y / pow(step, 2))
          continue;

        ccl_voxels_addr_[cell_id].push_back(voxel_addrs);
        Eigen::Vector3d voxel_color_free =
            Eigen::Vector3d(227.0 / 255.0, 187.0 / 255.0, 247.0 / 255.0);
        Eigen::Vector3d voxel_color_unknown =
            Eigen::Vector3d(230.0 / 255.0, 230.0 / 255.0, 230.0 / 255.0);

        if (free_unknown_state == 0) {
          free_unknown_states_and_centers_idx.push_back(
              std::make_pair(free_unknown_state, centers_free.size()));
          centers_free.push_back(Position(X_avg * map_server_->getResolution() + bbox_min.x(),
                                          Y_avg * map_server_->getResolution() + bbox_min.y(),
                                          height));
          ccl_voxels_color_[cell_id].push_back(voxel_color_free);
        } else if (free_unknown_state == 1) {
          free_unknown_states_and_centers_idx.push_back(
              std::make_pair(free_unknown_state, centers_unknown.size()));
          centers_unknown.push_back(Position(X_avg * map_server_->getResolution() + bbox_min.x(),
                                             Y_avg * map_server_->getResolution() + bbox_min.y(),
                                             height));
          ccl_voxels_color_[cell_id].push_back(voxel_color_unknown);
        } else
          CHECK(false) << "free_unknown_state is not set!";

        freeSpaces++;
        label++;
      }
    }
  }

  CHECK_EQ(ccl_voxels_addr_[cell_id].size(), free_unknown_states_and_centers_idx.size())
      << "CCL centers number mismatch!" << ccl_voxels_addr_[cell_id].size() << " vs "
      << free_unknown_states_and_centers_idx.size();

  int ccl_voxels_idx = 0;
  for (const std::pair<int, int> &free_unknown_state_and_center_idx :
       free_unknown_states_and_centers_idx) {

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    const std::unordered_set<int> &voxel_addr_set = ccl_voxels_addr_[cell_id][ccl_voxels_idx];
    for (const int &voxel_addr : voxel_addr_set) {
      Position voxel_pos = map_server_->getTSDF()->addressToPosition(voxel_addr);
      cloud->push_back(pcl::PointXYZ(voxel_pos.x(), voxel_pos.y(), voxel_pos.z()));
    }
    kdtree.setInputCloud(cloud);
    ccl_voxels_idx++;

    Position center = free_unknown_state_and_center_idx.first == 0
                          ? centers_free[free_unknown_state_and_center_idx.second]
                          : centers_unknown[free_unknown_state_and_center_idx.second];
    pcl::PointXYZ searchPoint(center.x(), center.y(), center.z());
    // ROS_INFO("[UniformGrid] Cell %d CCL center before: (%.2f, %.2f, %.2f)", cell_id, center.x(),
    //          center.y(), center.z());

    // K nearest neighbor search
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      pcl::PointXYZ nearest_point = cloud->points[pointIdxNKNSearch[0]];
      center = Position(nearest_point.x, nearest_point.y, nearest_point.z);
    } else {
      ROS_ERROR("[UniformGrid] K nearest neighbor search failed!");
    }

    if (free_unknown_state_and_center_idx.first == 0) {
      centers_free[free_unknown_state_and_center_idx.second] = center;
    } else if (free_unknown_state_and_center_idx.first == 1) {
      centers_unknown[free_unknown_state_and_center_idx.second] = center;
    } else {
      CHECK(false) << "free_unknown_state is not set!";
    }

    // ROS_INFO("[UniformGrid] Cell %d CCL center after: (%.2f, %.2f, %.2f)", cell_id, center.x(),
    //          center.y(), center.z());
  }

  CHECK_LE(freeSpaces, 10) << "Too many spaces in a cell!";
  // ROS_INFO("[UniformGrid] Found %d free spaces for cell %d", freeSpaces, cell_id);
}

// The L2 norm of the difference between two positions
double UniformGrid::getNormalCost(const Position &pos1, const Position &pos2) {
  return (pos1 - pos2).norm();
}

// The L2 norm of the difference between two positions with bias on x axis (tie breaking)
double UniformGrid::getXBiasedCost(const Position &pos1, const Position &pos2) {
  return sqrt(1.1 * (pos1.x() - pos2.x()) * (pos1.x() - pos2.x()) +
              (pos1.y() - pos2.y()) * (pos1.y() - pos2.y()) +
              (pos1.z() - pos2.z()) * (pos1.z() - pos2.z()));
}

// The cost of A* path from pos1 to pos2, passing only free voxels
double UniformGrid::getAStarCost(const Position &pos1, const Position &pos2,
                                 const Eigen::Vector3d &vel) {
  // Trick to avoid A* when the distance is too large
  // if ((pos1 - pos2).norm() > config_.cell_size_.x() * 2)
  //   return (pos1 - pos2).norm() * 2;

  double cost = 0.0;
  vector<Vector3d> path;

  cost = PathCostEvaluator::computeCost(pos1, pos2, 0.0, 0.0, vel, 0, path);

  // ROS_INFO("[UniformGrid] A* cost from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f): %.2f", pos1.x(),
  //          pos1.y(), pos1.z(), pos2.x(), pos2.y(), pos2.z(), cost);

  return cost;
}

// The cost of A* path from pos1 to pos2, passing unknown voxels is allowed
double UniformGrid::getAStarCostUnknown(const Position &pos1, const Position &pos2,
                                        const Eigen::Vector3d &vel) {
  // Trick to avoid A* when the distance is too large
  // if ((pos1 - pos2).norm() > config_.cell_size_.x() * 2)
  //   return (pos1 - pos2).norm() * 2;

  double cost = 0.0;
  vector<Vector3d> path;

  cost = PathCostEvaluator::computeCostUnknown(pos1, pos2, 0.0, 0.0, vel, 0, path);

  // ROS_INFO("[UniformGrid] A* cost from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f): %.2f", pos1.x(),
  //          pos1.y(), pos1.z(), pos2.x(), pos2.y(), pos2.z(), cost);

  return cost;
}

// The cost of A* path from pos1 to pos2 and yaw rotating cost, passing unknown voxels is allowed
double UniformGrid::getAStarCostYaw(const Position &pos1, const Position &pos2,
                                    const Eigen::Vector3d &vel, const double &yaw1,
                                    const double &yaw2) {
  double cost = 0.0;
  vector<Vector3d> path;

  cost = PathCostEvaluator::computeCostUnknown(pos1, pos2, yaw1, yaw2, vel, 0, path);

  return cost;
}

// The cost of local (within bbox) A* path from pos1 to pos2, passing only free voxels
double UniformGrid::getAStarCostBBox(const Position &pos1, const Position &pos2,
                                     const Position &bbox_min, const Position &bbox_max) {
  double cost = 0.0;
  vector<Vector3d> path;

  cost = PathCostEvaluator::computeCostBBox(pos1, pos2, 0.0, 0.0, Vector3d(0, 0, 0), 0, bbox_min,
                                            bbox_max, path);

  // ROS_INFO("[UniformGrid] A* cost from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f): %.2f", pos1.x(),
  //          pos1.y(), pos1.z(), pos2.x(), pos2.y(), pos2.z(), cost);

  return cost;
}

double UniformGrid::getAStarCostBBox(const Position &pos1, const Position &pos2,
                                     const Position &bbox_min, const Position &bbox_max,
                                     std::vector<Position> &path) {
  double cost = 0.0;

  cost = PathCostEvaluator::computeCostBBox(pos1, pos2, 0.0, 0.0, Vector3d(0, 0, 0), 0, bbox_min,
                                            bbox_max, path);

  // ROS_INFO("[UniformGrid] A* cost from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f): %.2f", pos1.x(),
  //          pos1.y(), pos1.z(), pos2.x(), pos2.y(), pos2.z(), cost);

  return cost;
}

// The cost of local (within bbox) A* path from pos1 to pos2, passing unknown voxels is allowed
double UniformGrid::getAStarCostBBoxUnknown(const Position &pos1, const Position &pos2,
                                            const Position &bbox_min, const Position &bbox_max) {
  double cost = 0.0;
  vector<Vector3d> path;

  cost = PathCostEvaluator::computeCostUnknownBBox(pos1, pos2, 0.0, 0.0, Vector3d(0, 0, 0), 0,
                                                   bbox_min, bbox_max, path);

  // ROS_INFO("[UniformGrid] A* cost from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f): %.2f", pos1.x(),
  //          pos1.y(), pos1.z(), pos2.x(), pos2.y(), pos2.z(), cost);

  return cost;
}

// The cost of local (within bbox) A* path from pos1 to pos2 and yaw rotating cost, passing unknown
double UniformGrid::getAStarCostBBoxUnknown(const Position &pos1, const Position &pos2,
                                            const Position &bbox_min, const Position &bbox_max,
                                            std::vector<Position> &path) {
  double cost = 0.0;

  cost = PathCostEvaluator::computeCostUnknownBBox(pos1, pos2, 0.0, 0.0, Vector3d(0, 0, 0), 0,
                                                   bbox_min, bbox_max, path);

  // ROS_INFO("[UniformGrid] A* cost from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f): %.2f", pos1.x(),
  //          pos1.y(), pos1.z(), pos2.x(), pos2.y(), pos2.z(), cost);

  return cost;
}

double UniformGrid::getAStarCostBBoxUnknownOnly(const Position &pos1, const Position &pos2,
                                                const Position &bbox_min,
                                                const Position &bbox_max) {
  double cost = 0.0;
  vector<Vector3d> path;

  cost = PathCostEvaluator::computeCostUnknownOnlyBBox(pos1, pos2, 0.0, 0.0, Vector3d(0, 0, 0), 0,
                                                       bbox_min, bbox_max, path);

  // ROS_INFO("[UniformGrid] A* cost from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f): %.2f", pos1.x(),
  //          pos1.y(), pos1.z(), pos2.x(), pos2.y(), pos2.z(), cost);

  return cost;
}

double UniformGrid::getAStarCostBBoxUnknownOnly(const Position &pos1, const Position &pos2,
                                                const Position &bbox_min, const Position &bbox_max,
                                                std::vector<Position> &path) {
  double cost = 0.0;

  cost = PathCostEvaluator::computeCostUnknownOnlyBBox(pos1, pos2, 0.0, 0.0, Vector3d(0, 0, 0), 0,
                                                       bbox_min, bbox_max, path);

  // ROS_INFO("[UniformGrid] A* cost from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f): %.2f", pos1.x(),
  //          pos1.y(), pos1.z(), pos2.x(), pos2.y(), pos2.z(), cost);

  return cost;
}

double UniformGrid::getAStarCostHGridGraph(const Position &pos1, const Position &pos2) {
  int cell_id1 = positionToGridCellId(pos1);
  int cell_id2 = positionToGridCellId(pos2);

  // Get A* cost on the grid sparse graph
  const double INF = 500.0;
  int N = cost_matrix_.size();
  vector<double> dist(N, INF);
  vector<int> prev(N, -1);
  std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                      std::greater<std::pair<double, int>>>
      pq;
  dist[cell_id1] = 0;
  pq.push(std::make_pair(0, cell_id1));
  while (!pq.empty()) {
    int u = pq.top().second;
    pq.pop();
    if (u == cell_id2) {
      break;
    }
    for (int v = 0; v < N; v++) {
      if (cost_matrix_[u][v] < INF && dist[u] + cost_matrix_[u][v] < dist[v]) {
        // Use the Euclidean distance as the heuristic function
        double h = (getCellCenter(v) - getCellCenter(cell_id2)).norm();
        dist[v] = dist[u] + cost_matrix_[u][v];
        prev[v] = u;
        pq.push(std::make_pair(dist[v] + h, v));
      }
    }
  }

  vector<int> path;
  for (int u = cell_id2; u != -1; u = prev[u]) {
    path.push_back(u);
  }
  reverse(path.begin(), path.end());

  return dist[cell_id2];
}

// cost_mat_id_to_cell_center_id is a map from cost matrix idx to cell id and center id
// A cell may have multiple centers, e.g. cell 10 has 4 centers, the cost matrix idx may be 11-14
void UniformGrid::calculateCostMatrixSingleThread(
    const Position &cur_pos, const Eigen::Vector3d &cur_vel, Eigen::MatrixXd &cost_matrix,
    std::map<int, std::pair<int, int>> &cost_mat_id_to_cell_center_id) {
  // ROS_INFO("[UniformGrid] Calculate cost matrix using multi-threading");

  int dim = 1;     // current position
  int mat_idx = 1; // skip 0 as it is reserved for current position

  for (const GridCell &grid_cell : uniform_grid_) {
    // Only free subspaces and unknown subspaces are considered in CP
    dim += grid_cell.centers_free_active_.size();
    dim += grid_cell.centers_unknown_active_.size();

    int cell_id = grid_cell.id_;

    // Cell center index: starting from free centers, then unknown centers
    for (int i = 0; i < (int)grid_cell.centers_free_active_.size(); i++) {
      cost_mat_id_to_cell_center_id[mat_idx] = std::make_pair(cell_id, i);
      mat_idx++;
    }

    for (int i = 0; i < (int)grid_cell.centers_unknown_active_.size(); i++) {
      cost_mat_id_to_cell_center_id[mat_idx] =
          std::make_pair(cell_id, i + grid_cell.centers_free_active_.size());
      mat_idx++;
    }
  }

  cost_matrix = Eigen::MatrixXd::Zero(dim, dim);

  // From current position to all zone centers
  mat_idx = 1;
  for (const GridCell &grid_cell : uniform_grid_) {
    double cost = 0.0;
    for (const Position &center_free : grid_cell.centers_free_active_) {
      if ((cur_pos - center_free).norm() > config_.hybrid_search_radius_) {
        cost = 1000.0 + (cur_pos - center_free).norm();
        // pair<int, int> cell_id_center_id_pair = cost_mat_id_to_cell_center_id[mat_idx];
        // int end_id = cell_id_center_id_pair.first * 10 + cell_id_center_id_pair.second;
      } else {
        cost = getAStarCostYaw(cur_pos, center_free, cur_vel, 0.0, 0.0);
        // if (cost > 499.0) {
        //   ROS_ERROR("[UniformGrid] Cost from current position to cell %d free center (%.2f, %.2f,
        //   "
        //             "%.2f) is high (unreachable)",
        //             grid_cell.id_, center_free.x(), center_free.y(), center_free.z());
        // }
      }

      // Add long distance panelty
      // if ((cur_pos - center_free).norm() > 5.0) {
      //   cost = cost * (cur_pos - center_free).norm() / 5.0;
      // }

      cost_matrix(0, mat_idx) = cost;

      CHECK_GT(cost, 1e-4) << "Zero cost from current position to cell " << grid_cell.id_
                           << " free center " << mat_idx - 1;
      // cost_matrix(mat_idx, 0) = 0 for ATSP

      mat_idx++;
    }

    for (const Position &center_unknown : grid_cell.centers_unknown_active_) {
      // Next zone should NOT be unknown
      cost = 1000.0 + (cur_pos - center_unknown).norm();

      cost_matrix(0, mat_idx) = cost * config_.unknown_penalty_factor_;
      // cost_matrix(mat_idx, 0) = 0 for ATSP

      CHECK_GT(cost, 1e-4) << "Zero cost from current position to cell " << grid_cell.id_
                           << " free center " << mat_idx - 1;

      mat_idx++;
    }
  }

  // Between all zone centers
  int mat_idx1 = 1;
  for (const GridCell &grid_cell1 : uniform_grid_) {
    // centers_free_active_ and centers_unknown_active_ of grid_cell1
    vector<Position> centers1;
    // centers1 centers_free_active_
    centers1.insert(centers1.end(), grid_cell1.centers_free_active_.begin(),
                    grid_cell1.centers_free_active_.end());
    // centers1 centers_unknown_active_
    centers1.insert(centers1.end(), grid_cell1.centers_unknown_active_.begin(),
                    grid_cell1.centers_unknown_active_.end());

    // Iterate over all centers in the grid_cell1
    for (int i = 0; i < (int)centers1.size(); i++) {
      int mat_idx2 = 1;
      // Iterate over all centers in the grid_cell2
      for (const GridCell &grid_cell2 : uniform_grid_) {
        // centers_free_active_ and centers_unknown_active_ of grid_cell2
        vector<Position> centers2;
        // centers2 centers_free_active_
        centers2.insert(centers2.end(), grid_cell2.centers_free_active_.begin(),
                        grid_cell2.centers_free_active_.end());
        // centers2 centers_unknown_active_
        centers2.insert(centers2.end(), grid_cell2.centers_unknown_active_.begin(),
                        grid_cell2.centers_unknown_active_.end());

        // Iterate over all centers in the grid_cell2
        for (int j = 0; j < (int)centers2.size(); j++) {
          // Skip half of the matrix
          if (mat_idx2 <= mat_idx1) {
            mat_idx2++;
            continue;
          }

          double cost = 0.0;
          int cell_center_id1 = -1, cell_center_id2 = -1;
          bool no_cg_search = false;
          if (i < (int)grid_cell1.centers_free_active_.size()) {
            if (grid_cell1.centers_free_active_idx_[i] == -1) {
              cell_center_id1 = grid_cell1.id_ * 10 + 0;
              no_cg_search = true;
            } else {
              cell_center_id1 = grid_cell1.id_ * 10 + grid_cell1.centers_free_active_idx_[i];
            }
          } else {
            cell_center_id1 =
                grid_cell1.id_ * 10 +
                grid_cell1.centers_unknown_active_idx_[i - grid_cell1.centers_free_active_.size()] +
                grid_cell1.centers_free_.size();
          }

          if (j < (int)grid_cell2.centers_free_active_.size()) {
            if (grid_cell2.centers_free_active_idx_[j] == -1) {
              cell_center_id2 = grid_cell2.id_ * 10 + 0;
              no_cg_search = true;
            } else {
              cell_center_id2 = grid_cell2.id_ * 10 + grid_cell2.centers_free_active_idx_[j];
            }
          } else {
            cell_center_id2 =
                grid_cell2.id_ * 10 +
                grid_cell2.centers_unknown_active_idx_[j - grid_cell2.centers_free_active_.size()] +
                grid_cell2.centers_free_.size();
          }

          CHECK_GE(cell_center_id1, 0) << "cell_center_id1: " << cell_center_id1;
          CHECK_GE(cell_center_id2, 0) << "cell_center_id2: " << cell_center_id2;

          if ((centers1[i] - centers2[j]).norm() > config_.hybrid_search_radius_) {
            if (no_cg_search) {
              // Invalid cell center id, use a large cost instead of CG search
              cost = 1000.0 + (centers1[i] - centers2[j]).norm();
            } else {
              std::vector<int> path;
              cost = connectivity_graph_->searchConnectivityGraphBFS(cell_center_id1,
                                                                     cell_center_id2, path);
              CHECK_GT(cost, 1e-4) << "Zero cost from cell " << grid_cell1.id_ << " center " << i
                                   << " to cell " << grid_cell2.id_ << " center " << j;
            }
          } else {
            cost = getAStarCostYaw(centers1[i], centers2[j], Eigen::Vector3d::Zero(), 0.0, 0.0);
            if (cost > 499.0) {
              // Try CG search
              if (no_cg_search) {
                cost = 1000.0 + (centers1[i] - centers2[j]).norm();
              } else {
                std::vector<int> path;
                cost = connectivity_graph_->searchConnectivityGraphBFS(cell_center_id1,
                                                                       cell_center_id2, path);
                CHECK_GT(cost, 1e-4) << "Zero cost from cell " << grid_cell1.id_ << " center " << i
                                     << " to cell " << grid_cell2.id_ << " center " << j;
              }
              if (cost < 1e-6) {
                // DEBUG output and check
                // print cell_center_id1 and cell_center_id2
                std::cout << "cell_center_id1: " << cell_center_id1
                          << ", cell_center_id2: " << cell_center_id2 << std::endl;
                // print grid_cell1.id_ and grid_cell2.id_
                std::cout << "grid_cell1.id_: " << grid_cell1.id_ << ", center id: " << i
                          << ", grid_cell2.id_: " << grid_cell2.id_ << ", center id: " << j
                          << std::endl;
                // print grid_cell1.centers_free_active_.size()
                std::cout << "grid_cell1.centers_free_active_.size(): "
                          << grid_cell1.centers_free_active_.size() << std::endl;
                // print grid_cell1.centers_free_active_idx_
                std::cout << "grid_cell1.centers_free_active_idx_: ";
                for (int i = 0; i < grid_cell1.centers_free_active_idx_.size(); i++) {
                  std::cout << grid_cell1.centers_free_active_idx_[i] << " ";
                }
                std::cout << std::endl;
                // print grid_cell1.centers_unknown_active_idx_
                std::cout << "grid_cell1.centers_unknown_active_idx_: ";
                for (int i = 0; i < grid_cell1.centers_unknown_active_idx_.size(); i++) {
                  std::cout << grid_cell1.centers_unknown_active_idx_[i] << " ";
                }
                std::cout << std::endl;
                // print grid_cell2.centers_free_active_.size()
                std::cout << "grid_cell2.centers_free_active_.size(): "
                          << grid_cell2.centers_free_active_.size() << std::endl;
                // print grid_cell2.centers_free_active_idx_
                std::cout << "grid_cell2.centers_free_active_idx_: ";
                for (int i = 0; i < grid_cell2.centers_free_active_idx_.size(); i++) {
                  std::cout << grid_cell2.centers_free_active_idx_[i] << " ";
                }
                std::cout << std::endl;
                // print grid_cell2.centers_unknown_active_idx_
                std::cout << "grid_cell2.centers_unknown_active_idx_: ";
                for (int i = 0; i < grid_cell2.centers_unknown_active_idx_.size(); i++) {
                  std::cout << grid_cell2.centers_unknown_active_idx_[i] << " ";
                }
                std::cout << std::endl;

                CHECK(false) << "Cost from cell " << grid_cell1.id_ << " center " << i
                             << " to cell " << grid_cell2.id_ << " center " << j
                             << " is high (unreachable) using A* but zero using CG, CG cost: "
                             << cost;
              }

              // if (cost > 499.0) {
              //   ROS_ERROR(
              //       "[UniformGrid] Cost from cell %d center (%.2f, %.2f, %.2f) to cell %d center
              //       "
              //       "(%.2f, %.2f, %.2f) is high (unreachable)",
              //       grid_cell1.id_, centers1[i].x(), centers1[i].y(), centers1[i].z(),
              //       grid_cell2.id_, centers2[j].x(), centers2[j].y(), centers2[j].z());
              // } else {
              //   ROS_WARN(
              //       "[UniformGrid] Cost from cell %d center (%.2f, %.2f, %.2f) to cell %d center
              //       "
              //       "(%.2f, %.2f, %.2f) is high (unreachable) using A* but reachable using CG, CG
              //       " "cost: %f", grid_cell1.id_, centers1[i].x(), centers1[i].y(),
              //       centers1[i].z(), grid_cell2.id_, centers2[j].x(), centers2[j].y(),
              //       centers2[j].z(), cost);
              // }
            }
          }

          if (i >= (int)grid_cell1.centers_free_active_.size() ||
              j >= (int)grid_cell2.centers_free_active_.size()) {
            cost *= config_.unknown_penalty_factor_;
            // ROS_INFO("[UniformGrid] penalize unknown center, new cost: %f", cost);
          }

          // Add long distance panelty
          // if ((centers1[i] - centers2[j]).norm() > 5.0) {
          //   cost = cost * (centers1[i] - centers2[j]).norm() / 5.0;
          // }

          cost_matrix(mat_idx1, mat_idx2) = cost_matrix(mat_idx2, mat_idx1) = cost;

          CHECK_GT(cost, 1e-6) << "Zero cost from cell " << grid_cell1.id_ << " center " << i
                               << " pos (" << centers1[i].x() << ", " << centers1[i].y() << ", "
                               << centers1[i].z() << ") to cell " << grid_cell2.id_ << " center "
                               << j << " pos (" << centers2[j].x() << ", " << centers2[j].y()
                               << ", " << centers2[j].z() << ")";
          mat_idx2++;
        }
      }
      mat_idx1++;
    }
  }
}

void UniformGrid::calculateCostMatrixMultiThread(
    const Position &cur_pos, const Eigen::Vector3d &cur_vel, const int &thread_num,
    Eigen::MatrixXd &cost_matrix,
    std::map<int, std::pair<int, int>> &cost_mat_id_to_cell_center_id) {
  CHECK(false) << "Astar path cost evaluation is not thread safe, use single thread";
  ROS_INFO("[UniformGrid] Calculate cost matrix using multi-threading");

  int dim = 1;     // current position
  int mat_idx = 1; // skip 0 as it is reserved for current position

  for (const GridCell &grid_cell : uniform_grid_) {
    // Only free subspaces and unknown subspaces are considered in CP
    dim += grid_cell.centers_free_active_.size();
    dim += grid_cell.centers_unknown_active_.size();

    int cell_id = grid_cell.id_;

    // Cell center index: starting from free centers, then unknown centers
    for (int i = 0; i < (int)grid_cell.centers_free_active_.size(); i++) {
      cost_mat_id_to_cell_center_id[mat_idx] = std::make_pair(cell_id, i);
      mat_idx++;
    }

    for (int i = 0; i < (int)grid_cell.centers_unknown_active_.size(); i++) {
      cost_mat_id_to_cell_center_id[mat_idx] =
          std::make_pair(cell_id, i + grid_cell.centers_free_active_.size());
      mat_idx++;
    }
  }

  ROS_INFO("[UniformGrid] Cost matrix dimension: %d", dim);

  cost_matrix = Eigen::MatrixXd::Zero(dim, dim);

  std::atomic<int> matrix_index(0);
  std::vector<std::thread> threads;

  for (int i = 0; i < thread_num; i++) {
    threads.push_back(std::thread([this, i, &matrix_index, &cost_mat_id_to_cell_center_id, &cur_pos,
                                   &cur_vel, &cost_matrix]() {
      // Init matrix index
      int index = matrix_index.fetch_add(1);
      int row = index / cost_matrix.cols();
      int col = index % cost_matrix.cols();

      while (row < cost_matrix.rows() && col < cost_matrix.cols()) {
        if (row < col) {
          if (row == 0) {
            // From current position to all zone centers
            std::pair<int, int> cell_id_center_id_pair = cost_mat_id_to_cell_center_id[index];
            int cell_id = cell_id_center_id_pair.first;
            int center_id = cell_id_center_id_pair.second;
            Position center;
            bool is_free_center = center_id < uniform_grid_[cell_id].centers_free_active_.size();
            if (is_free_center) {
              center = uniform_grid_[cell_id].centers_free_active_[center_id];
            } else {
              center =
                  uniform_grid_[cell_id]
                      .centers_unknown_active_[center_id -
                                               uniform_grid_[cell_id].centers_free_active_.size()];
            }

            double cost = 0.0;
            if ((cur_pos - center).norm() > config_.hybrid_search_radius_ || !is_free_center) {
              cost = 1000.0 + (cur_pos - center).norm();
            } else {
              // WARNING: getAStarCostYaw is not thread safe
              cost = getAStarCostYaw(cur_pos, center, cur_vel, 0.0, 0.0);
            }

            cost_matrix(0, index) = cost;
          } else {
            // Between all zone centers
            std::pair<int, int> cell_id_center_id_pair1 = cost_mat_id_to_cell_center_id[row];
            std::pair<int, int> cell_id_center_id_pair2 = cost_mat_id_to_cell_center_id[col];
            int cell_id1 = cell_id_center_id_pair1.first;
            int center_id1 = cell_id_center_id_pair1.second;
            int cell_id2 = cell_id_center_id_pair2.first;
            int center_id2 = cell_id_center_id_pair2.second;

            Position center1, center2;
            bool is_free_center1 = center_id1 < uniform_grid_[cell_id1].centers_free_active_.size();
            if (is_free_center1) {
              center1 = uniform_grid_[cell_id1].centers_free_active_[center_id1];
            } else {
              center1 =
                  uniform_grid_[cell_id1]
                      .centers_unknown_active_[center_id1 -
                                               uniform_grid_[cell_id1].centers_free_active_.size()];
            }
            bool is_free_center2 = center_id2 < uniform_grid_[cell_id2].centers_free_active_.size();
            if (is_free_center2) {
              center2 = uniform_grid_[cell_id2].centers_free_active_[center_id2];
            } else {
              center2 =
                  uniform_grid_[cell_id2]
                      .centers_unknown_active_[center_id2 -
                                               uniform_grid_[cell_id2].centers_free_active_.size()];
            }

            double cost = 0.0;
            if ((center1 - center2).norm() > config_.hybrid_search_radius_) {
              cost = 1000.0 + (center1 - center2).norm();
            } else {
              // WARNING: getAStarCostYaw is not thread safe
              cost = getAStarCostYaw(center1, center2, Eigen::Vector3d::Zero(), 0.0, 0.0);
            }

            if (!is_free_center1 || !is_free_center2) {
              cost *= config_.unknown_penalty_factor_;
            }

            cost_matrix(row, col) = cost_matrix(col, row) = cost;
          }
        }
        // Fetch next index
        index = matrix_index.fetch_add(1);
        row = index / cost_matrix.cols();
        col = index % cost_matrix.cols();
      }
    }));
  }

  std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));
}

void UniformGrid::getVisualizationLineLists(std::vector<Position> &list1,
                                            std::vector<Position> &list2) {
  for (const GridCell &grid_cell : uniform_grid_) {
    list1.insert(list1.end(), grid_cell.vertices1_.begin(), grid_cell.vertices1_.end());
    list2.insert(list2.end(), grid_cell.vertices2_.begin(), grid_cell.vertices2_.end());
  }
}

void UniformGrid::getVisualizationLineLists2D(std::vector<Position> &list1,
                                              std::vector<Position> &list2, double height) {
  for (const GridCell &grid_cell : uniform_grid_) {
    list1.insert(list1.end(), grid_cell.vertices1_.begin(), grid_cell.vertices1_.begin() + 4);
    list2.insert(list2.end(), grid_cell.vertices2_.begin(), grid_cell.vertices2_.begin() + 4);
  }

  // Set all z to height
  for (int i = 0; i < (int)list1.size(); i++) {
    list1[i].z() = height;
    list2[i].z() = height;
  }
}

void UniformGrid::getVisualizationConnectivityLineLists(std::vector<Position> &list1,
                                                        std::vector<Position> &list2) {
  for (const GridCell &grid_cell : uniform_grid_) {
    for (int nearby_cells_id : grid_cell.nearby_cells_ids_) {
      if (grid_cell.connectivity_matrixs_.find(nearby_cells_id) ==
          grid_cell.connectivity_matrixs_.end()) {
        continue;
      }

      std::vector<std::vector<bool>> connectivity_matrix =
          grid_cell.connectivity_matrixs_.find(nearby_cells_id)->second;
      for (int center1_count = 0; center1_count < (int)grid_cell.centers_free_.size();
           center1_count++) {
        for (int center2_count = 0;
             (int)center2_count < uniform_grid_[nearby_cells_id].centers_free_.size();
             center2_count++) {
          if (connectivity_matrix[center1_count][center2_count] == 0)
            continue;

          list1.push_back(grid_cell.centers_free_[center1_count]);
          list2.push_back(uniform_grid_[nearby_cells_id].centers_free_[center2_count]);
        }
      }
    }
  }
}

void UniformGrid::getVisualizationIdText(std::vector<std::string> &cell_ids,
                                         std::vector<Position> &cell_centers,
                                         std::vector<std::vector<Position>> &cell_centers_free,
                                         std::vector<std::vector<Position>> &cell_centers_unknown) {
  for (const GridCell &grid_cell : uniform_grid_) {
    cell_ids.push_back(std::to_string(grid_cell.id_));
    cell_centers.push_back(grid_cell.center_);
    cell_centers_free.push_back(grid_cell.centers_free_);
    cell_centers_unknown.push_back(grid_cell.centers_unknown_);
  }

  // auto increaseCentersZ = [&](std::vector<Position> &centers) {
  //   for (int i = 0; i < (int)centers.size(); i++) {
  //     centers[i].z() += 1.0;
  //   }
  // };

  // increaseCentersZ(cell_centers);
  // for (int i = 0; i < (int)cell_centers_free.size(); i++) {
  //   increaseCentersZ(cell_centers_free[i]);
  // }
  // for (int i = 0; i < (int)cell_centers_unknown.size(); i++) {
  //   increaseCentersZ(cell_centers_unknown[i]);
  // }
}

HierarchicalGrid::HierarchicalGrid() {
  ROS_ERROR("[HierarchicalGrid] Please initialize with a map server");
  CHECK(false) << "Please initialize hierarchical grid with a map server";
}

HierarchicalGrid::~HierarchicalGrid() {}

HierarchicalGrid::HierarchicalGrid(ros::NodeHandle &nh, voxel_mapping::MapServer::Ptr &map_server)
    : map_server_(map_server) {
  TicToc tic;

  // Load parameters
  nh.param("/hgrid/num_levels", config_.num_levels_, 1);
  nh.param("/hgrid/cell_size_max", config_.cell_size_max_, 5.0);
  nh.param("/hgrid/min_unknown_num_scale", config_.min_unknown_num_scale_, 0.1);
  nh.param("/hgrid/ccl_step", config_.ccl_step_, 2);
  nh.param("/hgrid/verbose", config_.verbose_, false);

  nh.param("/exploration_manager/hybrid_search_radius", config_.hybrid_search_radius_, 10.0);
  nh.param("/exploration_manager/unknown_penalty_factor", config_.unknown_penalty_factor_, 2.0);

  // ream map dim
  int map_dim = 0;
  nh.param("/map_config/map_dimension", map_dim, 2);
  CHECK(map_dim == 2 || map_dim == 3) << "Only 2D and 3D maps are supported";
  config_.use_2d_grids_ = (map_dim == 2);
  if (!config_.use_2d_grids_) {
    Position bbox_min, bbox_max;
    map_server_->getBox(bbox_min, bbox_max);
    double z_range = bbox_max.z() - bbox_min.z();
    if (z_range < 5.0)
      config_.cell_size_z_partition_num_ = 2;
    else if (z_range < 10.0)
      config_.cell_size_z_partition_num_ = 3;
    else if (z_range < 20.0)
      config_.cell_size_z_partition_num_ = 4;
    else
      config_.cell_size_z_partition_num_ = z_range / config_.cell_size_max_;
  }

  // Disable hybrid search for small map
  Position bbox_min, bbox_max;
  map_server_->getBox(bbox_min, bbox_max);
  double box_size = (bbox_max - bbox_min).prod();
  if (box_size < 1000.0)
    config_.hybrid_search_radius_ = std::numeric_limits<double>::max();

  // Initialize visualization publisher
  visulization_pubs_["Marker"] =
      nh.advertise<visualization_msgs::Marker>("/hgrid/visulization_marker", 100);
  visulization_pubs_["MarkerArray"] =
      nh.advertise<visualization_msgs::MarkerArray>("/hgrid/visulization_markerarray", 100);
  visulization_pubs_["PointCloud"] =
      nh.advertise<sensor_msgs::PointCloud2>("/hgrid/visulization_pointcloud", 10);

  CHECK_EQ(config_.num_levels_, 1) << "Other levels are not implemented yet";

  // Initialize multi-layer grids
  for (int i = 0; i < config_.num_levels_; ++i) {
    Position bbox_min, bbox_max;
    map_server_->getBox(bbox_min, bbox_max);

    // cell_size_ must be an integer multiple of map_resolution_
    if (std::fmod(config_.cell_size_max_, map_server_->getResolution()) > 1e-4 &&
        std::fabs(std::fmod(config_.cell_size_max_, map_server_->getResolution()) -
                  map_server_->getResolution()) > 1e-4) {
      // Round to a nearby integer multiple
      config_.cell_size_max_ = std::ceil(config_.cell_size_max_ / map_server_->getResolution()) *
                               map_server_->getResolution();
      ROS_WARN("[HierarchicalGrid] cell_size_max_ must be an integer multiple of map_resolution_, "
               "is now changed to %f",
               config_.cell_size_max_);
    }

    UniformGrid::Config ug_config;
    ug_config.level_ = i;
    ug_config.use_2d_grids_ = config_.use_2d_grids_;
    ug_config.bbox_min_ = bbox_min;
    ug_config.bbox_max_ = bbox_max;
    ug_config.cell_size_.x() = config_.cell_size_max_ / (std::pow(2, i));
    ug_config.cell_size_.y() = config_.cell_size_max_ / (std::pow(2, i));
    ug_config.cell_size_.z() = ug_config.use_2d_grids_ ? bbox_max.z() - bbox_min.z()
                                                       : (bbox_max.z() - bbox_min.z()) /
                                                             config_.cell_size_z_partition_num_;
    ug_config.num_cells_x_ =
        std::ceil((ug_config.bbox_max_.x() - ug_config.bbox_min_.x()) / ug_config.cell_size_.x());
    ug_config.num_cells_y_ =
        std::ceil((ug_config.bbox_max_.y() - ug_config.bbox_min_.y()) / ug_config.cell_size_.y());
    ug_config.num_cells_z_ = ug_config.use_2d_grids_
                                 ? 1
                                 : std::ceil((ug_config.bbox_max_.z() - ug_config.bbox_min_.z()) /
                                             ug_config.cell_size_.z());
    ug_config.num_cells_ = ug_config.num_cells_x_ * ug_config.num_cells_y_ * ug_config.num_cells_z_;
    ug_config.map_resolution_ = map_server_->getResolution();
    ug_config.num_voxels_per_cell_.x() =
        std::round(ug_config.cell_size_.x() / ug_config.map_resolution_);
    ug_config.num_voxels_per_cell_.y() =
        std::round(ug_config.cell_size_.y() / ug_config.map_resolution_);
    ug_config.num_voxels_per_cell_.z() =
        std::round(ug_config.cell_size_.z() / ug_config.map_resolution_);
    ug_config.min_unknown_num_ =
        config_.min_unknown_num_scale_ * ug_config.num_voxels_per_cell_.prod();
    ug_config.unknown_penalty_factor_ = config_.unknown_penalty_factor_;
    ug_config.hybrid_search_radius_ = config_.hybrid_search_radius_;
    ug_config.ccl_step_ = config_.ccl_step_;
    ug_config.verbose_ = config_.verbose_;

    if (config_.verbose_)
      ug_config.print();

    uniform_grids_.push_back(UniformGrid(nh, ug_config, map_server));
  }

  if (config_.verbose_)
    ROS_INFO("[HierarchicalGrid] Hierarchical grid initialized in %f s", tic.toc());

  ROS_INFO("[HierarchicalGrid] Hierarchical grid initialization finished");
}

void HierarchicalGrid::inputFrontiers(const std::vector<Position> &frontier_viewpoints,
                                      const std::vector<double> &frontier_yaws) {
  for (auto &uniform_grid : uniform_grids_)
    uniform_grid.inputFrontiers(frontier_viewpoints, frontier_yaws);
}

void HierarchicalGrid::updateHierarchicalGridFromVoxelMap(const Position &update_bbox_min,
                                                          const Position &update_bbox_max) {
  for (auto &uniform_grid : uniform_grids_) {
    TicToc tic;
    uniform_grid.updateGridsFromVoxelMap(update_bbox_min, update_bbox_max);
    if (config_.verbose_)
      ROS_INFO("[HierarchicalGrid] Update uniform grid from voxel map in %f s", tic.toc());
  }

  // ros::Time time_start = ros::Time::now();
  // publishGridsLayer();
  // publishGridsConnectivityGraphLayer();
  // ROS_INFO("[HierarchicalGrid] Publish grids layer in %f ms",
  //          (ros::Time::now() - time_start).toSec() * 1000.0);

  // publishGridsConnectivityLayer();
  // publishGridsCostMatrix();
}

void HierarchicalGrid::updateHierarchicalGridFrontierInfo(const Position &update_bbox_min,
                                                          const Position &update_bbox_max) {
  for (auto &uniform_grid : uniform_grids_) {
    TicToc tic;
    uniform_grid.updateGridsFrontierInfo(update_bbox_min, update_bbox_max);
    if (config_.verbose_)
      ROS_INFO("[HierarchicalGrid] Update uniform grid frontier info in %f s", tic.toc());
  }

  publishGridsLayer();
  publishGridsConnectivityGraphLayer();
  publishActiveFreeAndUnknownCenters();
}

void HierarchicalGrid::calculateCostMatrix2(
    const Position &cur_pos, const Eigen::Vector3d &cur_vel, const double &cur_yaw,
    const std::vector<Position> &last_path, Eigen::MatrixXd &cost_matrix,
    std::map<int, std::pair<int, int>> &cost_mat_id_to_cell_center_id) {
  // Calculate cost matrix for all active cells
  // TODO: use one-level grid for now, need to use multi-level grids (or cluster cells)
  TicToc tic;
  uniform_grids_[0].calculateCostMatrixSingleThread(cur_pos, cur_vel, cost_matrix,
                                                    cost_mat_id_to_cell_center_id);

  // int thread_num = 4;
  // Eigen::MatrixXd tmp_cost_matrix;
  // std::map<int, std::pair<int, int>> tmp_cost_mat_id_to_cell_center_id;
  // uniform_grids_[0].calculateCostMatrixMultiThread(cur_pos, cur_vel, thread_num, tmp_cost_matrix,
  //                                                  tmp_cost_mat_id_to_cell_center_id);

  if (config_.verbose_)
    ROS_INFO("[HierarchicalGrid] Calculate cost matrix 2 for level %d in %f s", 0, tic.toc());
}

void HierarchicalGrid::publishGridsLayer(const int &level) {
  if (visulization_pubs_["Marker"].getNumSubscribers() < 1 &&
      visulization_pubs_["MarkerArray"].getNumSubscribers() < 1 &&
      visulization_pubs_["PointCloud"].getNumSubscribers() < 1) {
    // ROS_INFO("[HierarchicalGrid] No subscribers for hgrid visualization");
    return;
  }

  if (level > config_.num_levels_ - 1) {
    ROS_ERROR("[HierarchicalGrid] Invalid level %d", level);
    return;
  }

  std::vector<Position> list1, list2;
  std::vector<std::string> cell_ids;
  std::vector<Position> cell_centers;
  std::vector<std::vector<Position>> cell_centers_free, cell_centers_unknown;
  if (config_.use_2d_grids_)
    uniform_grids_[level].getVisualizationLineLists2D(list1, list2, 1.0);
  else
    uniform_grids_[level].getVisualizationLineLists(list1, list2);
  uniform_grids_[level].getVisualizationIdText(cell_ids, cell_centers, cell_centers_free,
                                               cell_centers_unknown);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "hgrid_layer" + std::to_string(level);
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  for (unsigned int i = 0; i < list1.size(); ++i) {
    geometry_msgs::Point p1, p2;
    p1.x = list1[i].x();
    p1.y = list1[i].y();
    p1.z = list1[i].z();

    p2.x = list2[i].x();
    p2.y = list2[i].y();
    p2.z = list2[i].z();

    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  visulization_pubs_["Marker"].publish(marker);

  // Publish grid id at is center position
  visualization_msgs::MarkerArray marker_ids;
  visualization_msgs::Marker marker_id, marker_id_delete;
  marker_id.header.frame_id = "world";
  marker_id.header.stamp = ros::Time::now();
  marker_id.ns = "hgrid_layer" + std::to_string(level) + "_id";

  marker_id_delete.header.frame_id = "world";
  marker_id_delete.header.stamp = ros::Time::now();
  marker_id_delete.ns = "hgrid_layer" + std::to_string(level) + "_id";

  // Clear all previous markers
  for (unsigned int i = 0; i < cell_ids.size() * 10 + cell_ids.size(); ++i) {
    marker_id.id = i;
    marker_id.action = visualization_msgs::Marker::DELETE;
    marker_ids.markers.push_back(marker_id);
  }
  marker_ids.markers.push_back(marker_id);
  visulization_pubs_["MarkerArray"].publish(marker_ids);
  marker_ids.markers.clear();

  marker_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_id.action = visualization_msgs::Marker::ADD;

  marker_id.scale.x = 1.0;
  marker_id.scale.y = 1.0;
  marker_id.scale.z = 1.0;

  marker_id.lifetime = ros::Duration();

  for (unsigned int i = 0; i < cell_ids.size(); ++i) {
    // Push back cell center of free cells
    int centers_cnt = 0;
    for (int j = 0; j < cell_centers_free[i].size(); ++j) {
      marker_id.id = i * 10 + cell_ids.size() + centers_cnt;
      marker_id.text = cell_ids[i] + "_" + std::to_string(centers_cnt);
      ++centers_cnt;

      marker_id.pose.position.x = cell_centers_free[i][j].x();
      marker_id.pose.position.y = cell_centers_free[i][j].y();
      marker_id.pose.position.z = cell_centers_free[i][j].z() + 0.5;

      double scale = 0.4;
      marker_id.scale.x = scale;
      marker_id.scale.y = scale;
      marker_id.scale.z = scale;

      // Blue
      marker_id.color.r = 0.0;
      marker_id.color.g = 0.0;
      marker_id.color.b = 1.0;
      marker_id.color.a = 1.0;

      marker_ids.markers.push_back(marker_id);
    }

    // Push back cell center of unknown cells
    for (int j = 0; j < cell_centers_unknown[i].size(); ++j) {
      marker_id.id = i * 10 + cell_ids.size() + centers_cnt;
      marker_id.text = cell_ids[i] + "_" + std::to_string(centers_cnt);
      ++centers_cnt;

      marker_id.pose.position.x = cell_centers_unknown[i][j].x();
      marker_id.pose.position.y = cell_centers_unknown[i][j].y();
      marker_id.pose.position.z = cell_centers_unknown[i][j].z() + 0.5;

      double scale = 0.4;
      marker_id.scale.x = scale;
      marker_id.scale.y = scale;
      marker_id.scale.z = scale;

      // Green
      marker_id.color.r = 0.0;
      marker_id.color.g = 1.0;
      marker_id.color.b = 0.0;
      marker_id.color.a = 1.0;

      marker_ids.markers.push_back(marker_id);
    }

    // centers_cnt < 10; fill with delete marker
    for (int j = centers_cnt; j < 10; ++j) {
      marker_id_delete.id = i * 10 + cell_ids.size() + j;
      marker_id_delete.action = visualization_msgs::Marker::DELETE;
      marker_ids.markers.push_back(marker_id_delete);
    }
  }

  visulization_pubs_["MarkerArray"].publish(marker_ids);

  // Publish CCL voxels
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  for (unsigned int i = 0; i < cell_ids.size(); ++i) {
    VoxelIndex cell_bbox_min, cell_bbox_max;
    uniform_grids_[level].getCellBBoxIndex(i, cell_bbox_min, cell_bbox_max);

    for (int j = 0; j < uniform_grids_[0].ccl_voxels_addr_[i].size(); ++j) {
      std::unordered_set<int> &voxel_addrs = uniform_grids_[0].ccl_voxels_addr_[i][j];
      Eigen::Vector3d &voxel_color = uniform_grids_[0].ccl_voxels_color_[i][j];
      for (auto &addr : voxel_addrs) {
        pcl::PointXYZRGB point;
        Position pos = map_server_->getTSDF()->addressToPosition(addr);
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();
        point.r = voxel_color.x() * 255;
        point.g = voxel_color.y() * 255;
        point.b = voxel_color.z() * 255;

        cloud.push_back(point);

        // inflate the voxel by ccl step
        VoxelIndex voxel_index = map_server_->getTSDF()->addressToIndex(addr);
        if (config_.ccl_step_ > 1) {
          for (int dx = 0; dx < config_.ccl_step_; ++dx) {
            for (int dy = 0; dy < config_.ccl_step_; ++dy) {
              if (dx == 0 && dy == 0)
                continue;
              VoxelIndex voxel_index_inflated = voxel_index + VoxelIndex(dx, dy, 0);
              if (voxel_index_inflated.x() < cell_bbox_min.x() ||
                  voxel_index_inflated.y() < cell_bbox_min.y() ||
                  voxel_index_inflated.z() < cell_bbox_min.z() ||
                  voxel_index_inflated.x() > cell_bbox_max.x() ||
                  voxel_index_inflated.y() > cell_bbox_max.y() ||
                  voxel_index_inflated.z() > cell_bbox_max.z())
                continue;
              Position pos_inflated = map_server_->getTSDF()->indexToPosition(voxel_index_inflated);
              pcl::PointXYZRGB point_inflated;
              point_inflated.x = pos_inflated.x();
              point_inflated.y = pos_inflated.y();
              point_inflated.z = pos_inflated.z();
              point_inflated.r = voxel_color.x() * 255;
              point_inflated.g = voxel_color.y() * 255;
              point_inflated.b = voxel_color.z() * 255;
              cloud.push_back(point_inflated);
            }
          }
        }
      }
    }
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  visulization_pubs_["PointCloud"].publish(cloud_msg);
}

void HierarchicalGrid::publishGridsConnectivityLayer(const int &level) {
  if (visulization_pubs_["Marker"].getNumSubscribers() < 1) {
    // ROS_INFO("[HierarchicalGrid] No subscribers for hgrid visualization");
    return;
  }

  if (level > config_.num_levels_ - 1) {
    ROS_ERROR("[HierarchicalGrid] Invalid level %d", level);
    return;
  }

  std::vector<Position> list1, list2;
  uniform_grids_[level].getVisualizationConnectivityLineLists(list1, list2);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "hgrid_layer" + std::to_string(level) + "_connectivity";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  for (unsigned int i = 0; i < list1.size(); ++i) {
    geometry_msgs::Point p1, p2;
    p1.x = list1[i].x();
    p1.y = list1[i].y();
    p1.z = list1[i].z();

    p2.x = list2[i].x();
    p2.y = list2[i].y();
    p2.z = list2[i].z();

    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  visulization_pubs_["Marker"].publish(marker);
}

void HierarchicalGrid::publishGridsConnectivityGraphLayer(const int &level) {
  if (visulization_pubs_["MarkerArray"].getNumSubscribers() < 1) {
    // ROS_INFO("[HierarchicalGrid] No subscribers for hgrid visualization");
    return;
  }

  const Eigen::Vector3d color_unknown =
      Eigen::Vector3d(118.0 / 255.0, 113.0 / 255.0, 113.0 / 255.0);
  const Eigen::Vector3d color_free = Eigen::Vector3d(255.0 / 255.0, 60.0 / 255.0, 60.0 / 255.0);
  const Eigen::Vector3d color_portal = Eigen::Vector3d(0.0 / 255.0, 100.0 / 255.0, 255.0 / 255.0);
  const Eigen::Vector3d color_cost = Eigen::Vector3d(0.0 / 255.0, 0.0 / 255.0, 0.0 / 255.0);
  const double scale = 0.2;

  visualization_msgs::MarkerArray marker_array_connectivity_graph;
  visualization_msgs::Marker marker_unknown, marker_free, marker_portal, marker_cost;
  marker_unknown.header.frame_id = "world";
  marker_unknown.header.stamp = ros::Time::now();
  marker_unknown.ns = "hgrid_layer" + std::to_string(level) + "_connectivity_graph";
  marker_unknown.id = 0;
  marker_unknown.type = visualization_msgs::Marker::LINE_LIST;
  marker_unknown.action = visualization_msgs::Marker::ADD;
  marker_unknown.scale.x = scale;
  marker_unknown.scale.y = scale;
  marker_unknown.scale.z = scale;
  marker_unknown.lifetime = ros::Duration();
  marker_unknown.pose.orientation.w = 1.0;
  marker_unknown.pose.orientation.x = 0.0;
  marker_unknown.pose.orientation.y = 0.0;
  marker_unknown.pose.orientation.z = 0.0;
  marker_unknown.color.r = color_unknown.x();
  marker_unknown.color.g = color_unknown.y();
  marker_unknown.color.b = color_unknown.z();
  marker_unknown.color.a = 1.0;

  marker_free.header.frame_id = "world";
  marker_free.header.stamp = ros::Time::now();
  marker_free.ns = "hgrid_layer" + std::to_string(level) + "_connectivity_graph";
  marker_free.id = 1;
  marker_free.type = visualization_msgs::Marker::LINE_LIST;
  marker_free.action = visualization_msgs::Marker::ADD;
  marker_free.scale.x = scale;
  marker_free.scale.y = scale;
  marker_free.scale.z = scale;
  marker_free.lifetime = ros::Duration();
  marker_free.pose.orientation.w = 1.0;
  marker_free.pose.orientation.x = 0.0;
  marker_free.pose.orientation.y = 0.0;
  marker_free.pose.orientation.z = 0.0;
  marker_free.color.r = color_free.x();
  marker_free.color.g = color_free.y();
  marker_free.color.b = color_free.z();
  marker_free.color.a = 1.0;

  marker_portal.header.frame_id = "world";
  marker_portal.header.stamp = ros::Time::now();
  marker_portal.ns = "hgrid_layer" + std::to_string(level) + "_connectivity_graph";
  marker_portal.id = 2;
  marker_portal.type = visualization_msgs::Marker::LINE_LIST;
  marker_portal.action = visualization_msgs::Marker::ADD;
  marker_portal.scale.x = scale;
  marker_portal.scale.y = scale;
  marker_portal.scale.z = scale;
  marker_portal.lifetime = ros::Duration();
  marker_portal.pose.orientation.w = 1.0;
  marker_portal.pose.orientation.x = 0.0;
  marker_portal.pose.orientation.y = 0.0;
  marker_portal.pose.orientation.z = 0.0;
  marker_portal.color.r = color_portal.x();
  marker_portal.color.g = color_portal.y();
  marker_portal.color.b = color_portal.z();
  marker_portal.color.a = 1.0;

  marker_cost.header.frame_id = "world";
  marker_cost.header.stamp = ros::Time::now();
  marker_cost.ns = "hgrid_layer" + std::to_string(level) + "_connectivity_graph";
  marker_cost.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_cost.action = visualization_msgs::Marker::ADD;
  marker_cost.scale.x = 0.5;
  marker_cost.scale.y = 0.5;
  marker_cost.scale.z = 0.5;
  marker_cost.lifetime = ros::Duration();
  marker_cost.pose.orientation.w = 1.0;
  marker_cost.pose.orientation.x = 0.0;
  marker_cost.pose.orientation.y = 0.0;
  marker_cost.pose.orientation.z = 0.0;
  marker_cost.color.r = color_cost.x();
  marker_cost.color.g = color_cost.y();
  marker_cost.color.b = color_cost.z();
  marker_cost.color.a = 1.0;

  std::vector<std::pair<Position, Position>> lines;
  std::vector<std::vector<Position>> paths;
  std::vector<ConnectivityEdge::TYPE> types;
  std::vector<double> costs;
  uniform_grids_[level].connectivity_graph_->getFullConnectivityGraph(lines, types, costs);
  for (unsigned int i = 0; i < lines.size(); ++i) {
    if (types[i] == ConnectivityEdge::TYPE::UNKNOWN) {
      geometry_msgs::Point p1, p2;
      p1.x = lines[i].first.x();
      p1.y = lines[i].first.y();
      p1.z = lines[i].first.z();

      p2.x = lines[i].second.x();
      p2.y = lines[i].second.y();
      p2.z = lines[i].second.z();

      marker_unknown.points.push_back(p1);
      marker_unknown.points.push_back(p2);
    } else if (types[i] == ConnectivityEdge::TYPE::FREE) {
      geometry_msgs::Point p1, p2;
      p1.x = lines[i].first.x();
      p1.y = lines[i].first.y();
      p1.z = lines[i].first.z();

      p2.x = lines[i].second.x();
      p2.y = lines[i].second.y();
      p2.z = lines[i].second.z();

      marker_free.points.push_back(p1);
      marker_free.points.push_back(p2);
    } else if (types[i] == ConnectivityEdge::TYPE::PORTAL) {
      geometry_msgs::Point p1, p2;
      p1.x = lines[i].first.x();
      p1.y = lines[i].first.y();
      p1.z = lines[i].first.z();

      p2.x = lines[i].second.x();
      p2.y = lines[i].second.y();
      p2.z = lines[i].second.z();

      marker_portal.points.push_back(p1);
      marker_portal.points.push_back(p2);
    } else {
      CHECK(false) << "Invalid connectivity edge type";
    }

    // Add cost text
    geometry_msgs::Point p;
    p.x = (lines[i].first.x() + lines[i].second.x()) / 2.0;
    p.y = (lines[i].first.y() + lines[i].second.y()) / 2.0;
    p.z = (lines[i].first.z() + lines[i].second.z()) / 2.0 + 0.5;
    marker_cost.id = i + 3;
    marker_cost.text = std::to_string(costs[i]);
    marker_cost.pose.position = p;
    marker_array_connectivity_graph.markers.push_back(marker_cost);
  }

  marker_array_connectivity_graph.markers.push_back(marker_unknown);
  marker_array_connectivity_graph.markers.push_back(marker_free);
  marker_array_connectivity_graph.markers.push_back(marker_portal);
  visulization_pubs_["MarkerArray"].publish(marker_array_connectivity_graph);
}

void HierarchicalGrid::publishGridsCostMatrix(const int &level) {
  if (visulization_pubs_["MarkerArray"].getNumSubscribers() < 1) {
    // ROS_INFO("[HierarchicalGrid] No subscribers for hgrid visualization");
    return;
  }

  const std::vector<std::vector<double>> cost_matrix = uniform_grids_[level].getCostMatrix();

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "hgrid_layer" + std::to_string(level) + "_cost_matrix";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  marker.color.r = 0.75;
  marker.color.g = 0.75;
  marker.color.b = 0.75;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  for (unsigned int i = 0; i < uniform_grids_[level].getNumCells(); ++i) {
    std::vector<int> nearby_cell_ids;
    uniform_grids_[level].getNearbyCellIds(i, nearby_cell_ids);

    for (unsigned int j = 0; j < nearby_cell_ids.size(); ++j) {
      marker.id = marker.id + 1;
      marker.text = std::to_string(cost_matrix[i][nearby_cell_ids[j]]);

      Position cell_center = (uniform_grids_[level].getCellCenter(i) +
                              uniform_grids_[level].getCellCenter(nearby_cell_ids[j])) /
                             2.0;
      marker.pose.position.x = cell_center.x();
      marker.pose.position.y = cell_center.y();
      marker.pose.position.z = cell_center.z();

      marker_array.markers.push_back(marker);
    }
  }

  visulization_pubs_["MarkerArray"].publish(marker_array);
}

void HierarchicalGrid::publishActiveFreeAndUnknownCenters(const int &level) {
  if (visulization_pubs_["MarkerArray"].getNumSubscribers() < 1) {
    // ROS_INFO("[HierarchicalGrid] No subscribers for hgrid visualization");
    return;
  }

  const Eigen::Vector3d color_free = Eigen::Vector3d(255.0 / 255.0, 60.0 / 255.0, 60.0 / 255.0);
  const Eigen::Vector3d color_unknown =
      Eigen::Vector3d(118.0 / 255.0, 113.0 / 255.0, 113.0 / 255.0);
  const double scale = 0.5;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "hgrid_layer" + std::to_string(level) + "_active_free_unknown_centers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  int id = 0;
  for (unsigned int i = 0; i < uniform_grids_[level].getNumCells(); ++i) {
    // active free centers
    for (const Position &center : uniform_grids_[level].uniform_grid_[i].centers_free_) {
      marker.id = id++;
      marker.color.r = color_free.x();
      marker.color.g = color_free.y();
      marker.color.b = color_free.z();
      marker.color.a = 1.0;
      marker.pose.position.x = center.x();
      marker.pose.position.y = center.y();
      marker.pose.position.z = center.z();
      marker_array.markers.push_back(marker);
    }

    // unknown centers
    for (const Position &center : uniform_grids_[level].uniform_grid_[i].centers_unknown_) {
      marker.id = id++;
      marker.color.r = color_unknown.x();
      marker.color.g = color_unknown.y();
      marker.color.b = color_unknown.z();
      marker.color.a = 1.0;
      marker.pose.position.x = center.x();
      marker.pose.position.y = center.y();
      marker.pose.position.z = center.z();
      marker_array.markers.push_back(marker);
    }
  }

  static unsigned int last_num = 0;
  if (marker_array.markers.size() < last_num) {
    // Fill the rest with delete marker
    for (unsigned int i = marker_array.markers.size(); i < last_num; ++i) {
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETE;
      marker_array.markers.push_back(marker);
    }
  }
  last_num = marker_array.markers.size();

  if (marker_array.markers.size() > 0)
    visulization_pubs_["MarkerArray"].publish(marker_array);
}

} // namespace fast_planner