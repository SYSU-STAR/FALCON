#include <sstream>

#include "pathfinding/astar.h"

using namespace std;
using namespace Eigen;

namespace fast_planner {
Astar::Astar() {}

Astar::~Astar() {
  for (int i = 0; i < (int)config_.allocate_num_; i++)
    delete path_node_pool_[i];
}

void Astar::init(ros::NodeHandle &nh, const MapServer::Ptr &map_server) {
  nh.param("/astar/lambda_heuristic", config_.lambda_heuristic_, -1.0);
  nh.param("/astar/allocate_num", config_.allocate_num_, -1);
  nh.param("/astar/epsilon", config_.epsilon_, -1.0);
  nh.param("/astar/profile/default/resolution", config_.default_resolution_, -1.0);
  nh.param("/astar/profile/default/max_search_time", config_.default_max_search_time_, -1.0);
  nh.param("/astar/profile/coarse/resolution", config_.coarse_resolution_, -1.0);
  nh.param("/astar/profile/coarse/max_search_time", config_.coarse_max_search_time_, -1.0);
  nh.param("/astar/profile/coarse2/resolution", config_.coarse2_resolution_, -1.0);
  nh.param("/astar/profile/coarse2/max_search_time", config_.coarse2_max_search_time_, -1.0);
  nh.param("/astar/profile/medium/resolution", config_.medium_resolution_, -1.0);
  nh.param("/astar/profile/medium/max_search_time", config_.medium_max_search_time_, -1.0);
  nh.param("/astar/profile/fine/resolution", config_.fine_resolution_, -1.0);
  nh.param("/astar/profile/fine/max_search_time", config_.fine_max_search_time_, -1.0);
  nh.param("/astar/verbose", config_.verbose_, false);

  this->map_server_ = map_server;
  setProfile(PROFILE::DEFAULT);
  config_.tie_breaker_ = 1.0 + config_.epsilon_;
  config_.map_origin_ = map_server_->getOrigin();

  if (config_.verbose_) {
    config_.print();
  }

  path_node_pool_.resize(config_.allocate_num_);
  for (int i = 0; i < (int)config_.allocate_num_; i++) {
    path_node_pool_[i] = new AstarNode;
  }
  use_node_num_ = 0;
  iter_num_ = 0;

  ray_caster_.reset(new RayCaster);
  ray_caster_->setParams(map_server_->getResolution(), map_server_->getOrigin());
}

// Dynamically change resolution to adapt to different environment
void Astar::setResolution(const double &res) {
  config_.resolution_ = res;
  config_.resolution_inverse_ = 1.0 / config_.resolution_;
}

void Astar::setMaxSearchTime(const double &t) { config_.max_search_time_ = t; }

void Astar::setProfile(const PROFILE &p) {
  switch (p) {
  case PROFILE::DEFAULT:
    config_.resolution_ = config_.default_resolution_;
    config_.resolution_inverse_ = 1.0 / config_.default_resolution_;
    config_.max_search_time_ = config_.default_max_search_time_;
    break;
  case PROFILE::COARSE:
    config_.resolution_ = config_.coarse_resolution_;
    config_.resolution_inverse_ = 1.0 / config_.coarse_resolution_;
    config_.max_search_time_ = config_.coarse_max_search_time_;
    break;
  case PROFILE::COARSE2:
    config_.resolution_ = config_.coarse2_resolution_;
    config_.resolution_inverse_ = 1.0 / config_.coarse2_resolution_;
    config_.max_search_time_ = config_.coarse2_max_search_time_;
    break;
  case PROFILE::MEDIUM:
    config_.resolution_ = config_.medium_resolution_;
    config_.resolution_inverse_ = 1.0 / config_.medium_resolution_;
    config_.max_search_time_ = config_.medium_max_search_time_;
    break;
  case PROFILE::FINE:
    config_.resolution_ = config_.fine_resolution_;
    config_.resolution_inverse_ = 1.0 / config_.fine_resolution_;
    config_.max_search_time_ = config_.fine_max_search_time_;
    break;
  default:
    CHECK(false) << "Invalid astar profile";
    break;
  }
}

int Astar::search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt, const MODE &mode,
                  const Position &bbox_min, const Position &bbox_max) {
  auto isInBox = [&](const Position &p) {
    return p.x() > bbox_min.x() && p.x() < bbox_max.x() && p.y() > bbox_min.y() &&
           p.y() < bbox_max.y() && p.z() > bbox_min.z() && p.z() < bbox_max.z();
  };

  auto isValidVoxel = [&](const Position &p) {
    bool is_valid = false;

    VoxelIndex idx = map_server_->getOccupancyGrid()->positionToIndex(p);
    voxel_mapping::OccupancyType voxel_type = map_server_->getOccupancyGrid()->getVoxel(idx).value;

    switch (mode) {
    case Astar::MODE::FREE_ONLY:
    case Astar::MODE::FREE_ONLY_BBOX:
      is_valid = (voxel_type == voxel_mapping::OccupancyType::FREE && isInBox(p));
      break;
    case Astar::MODE::FREE_UNKNOWN:
    case Astar::MODE::FREE_UNKNOWN_BBOX:
      is_valid = (voxel_type == voxel_mapping::OccupancyType::FREE ||
                  voxel_type == voxel_mapping::OccupancyType::UNKNOWN) &&
                 isInBox(p);
      break;
    case Astar::MODE::UNKNOWN_ONLY:
    case Astar::MODE::UNKNOWN_ONLY_BBOX:
      is_valid = (voxel_type == voxel_mapping::OccupancyType::UNKNOWN && isInBox(p));
      break;
    default:
      CHECK(false) << "Invalid mode type for astar search";
      break;
    }

    return is_valid;
  };

  AstarNode::Ptr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = config_.lambda_heuristic_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) && cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    // Check if current node can reach the end point without collision
    bool safe = true;
    Vector3d dir = end_pt - cur_node->position;
    double len = dir.norm();
    dir.normalize();
    for (double l = 0.1; l < len; l += 0.1) {
      Vector3d ckpt = cur_node->position + l * dir;
      if (!isValidVoxel(ckpt)) {
        safe = false;
        break;
      }
    }
    if (safe) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > config_.max_search_time_) {
      // ROS_WARN("[AStar] Early terminated, duration: %f", (ros::Time::now() - t1).toSec());
      // early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -config_.resolution_ + config_.epsilon_;
         dx <= config_.resolution_ + config_.epsilon_; dx += config_.resolution_)
      for (double dy = -config_.resolution_ + config_.epsilon_;
           dy <= config_.resolution_ + config_.epsilon_; dy += config_.resolution_)
        for (double dz = -config_.resolution_ + config_.epsilon_;
             dz <= config_.resolution_ + config_.epsilon_; dz += config_.resolution_) {
          step << dx, dy, dz;
          if (step.norm() < config_.epsilon_)
            continue;
          nbr_pos = cur_pos + step;
          // Check safety
          if (!isValidVoxel(nbr_pos)) {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (!isValidVoxel(ckpt)) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) {
            continue;
          }

          AstarNode::Ptr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == config_.allocate_num_) {
              ROS_WARN("[AStar] Run out of node pool. Duration: %f",
                       (ros::Time::now() - t1).toSec());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + config_.lambda_heuristic_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

int Astar::search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) {
  AstarNode::Ptr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = config_.lambda_heuristic_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) && cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    // if (abs(cur_node->index(0) - end_index(0)) <= 1 &&
    //     abs(cur_node->index(1) - end_index(1)) <= 1 &&
    //     abs(cur_node->index(2) - end_index(2)) <= 1) {
    // Check if current node can reach the end point without collision
    if (true) {
      bool safe = true;
      Vector3d dir = end_pt - cur_node->position;
      double len = dir.norm();
      dir.normalize();
      for (double l = 0.1; l < len; l += 0.1) {
        Vector3d ckpt = cur_node->position + l * dir;
        if (map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                voxel_mapping::OccupancyType::OCCUPIED ||
            map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                voxel_mapping::OccupancyType::UNKNOWN) {
          safe = false;
          break;
        }
      }
      if (safe) {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > config_.max_search_time_) {
      // ROS_WARN("[AStar] Early terminated, duration: %f", (ros::Time::now() - t1).toSec());
      // early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -config_.resolution_ + config_.epsilon_;
         dx <= config_.resolution_ + config_.epsilon_; dx += config_.resolution_)
      for (double dy = -config_.resolution_ + config_.epsilon_;
           dy <= config_.resolution_ + config_.epsilon_; dy += config_.resolution_)
        for (double dz = -config_.resolution_ + config_.epsilon_;
             dz <= config_.resolution_ + config_.epsilon_; dz += config_.resolution_) {
          step << dx, dy, dz;
          if (step.norm() < config_.epsilon_)
            continue;
          nbr_pos = cur_pos + step;
          // Check safety
          if (!map_server_->getTSDF()->isInBox(nbr_pos)) {
            continue;
          }

          if (map_server_->getOccupancyGrid()->getVoxel(nbr_pos).value ==
                  voxel_mapping::OccupancyType::OCCUPIED ||
              map_server_->getOccupancyGrid()->getVoxel(nbr_pos).value ==
                  voxel_mapping::OccupancyType::UNKNOWN) {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                    voxel_mapping::OccupancyType::OCCUPIED ||
                map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                    voxel_mapping::OccupancyType::UNKNOWN) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) {
            continue;
          }

          AstarNode::Ptr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == config_.allocate_num_) {
              ROS_WARN("[AStar] Run out of node pool. Duration: %f",
                       (ros::Time::now() - t1).toSec());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + config_.lambda_heuristic_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

int Astar::searchBBox(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
                      const Eigen::Vector3d &bbox_min, const Eigen::Vector3d &bbox_max) {
  AstarNode::Ptr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = config_.lambda_heuristic_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) && cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    if (abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 &&
        abs(cur_node->index(2) - end_index(2)) <= 1) {
      bool safe = true;
      Vector3d dir = end_pt - cur_node->position;
      double len = dir.norm();
      dir.normalize();
      for (double l = 0.1; l < len; l += 0.1) {
        Vector3d ckpt = cur_node->position + l * dir;
        if (map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                voxel_mapping::OccupancyType::OCCUPIED ||
            map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                voxel_mapping::OccupancyType::UNKNOWN) {
          safe = false;
          break;
        }
      }
      if (safe) {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > config_.max_search_time_) {
      // ROS_WARN("[AStar] Early terminated, duration: %f", (ros::Time::now() - t1).toSec());
      // early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -config_.resolution_ + config_.epsilon_;
         dx <= config_.resolution_ + config_.epsilon_; dx += config_.resolution_)
      for (double dy = -config_.resolution_ + config_.epsilon_;
           dy <= config_.resolution_ + config_.epsilon_; dy += config_.resolution_)
        for (double dz = -config_.resolution_ + config_.epsilon_;
             dz <= config_.resolution_ + config_.epsilon_; dz += config_.resolution_) {
          step << dx, dy, dz;
          if (step.norm() < config_.epsilon_)
            continue;
          nbr_pos = cur_pos + step;
          // Check safety
          auto isInBox = [&](const Eigen::Vector3d &pos) {
            return pos(0) >= bbox_min(0) && pos(0) <= bbox_max(0) && pos(1) >= bbox_min(1) &&
                   pos(1) <= bbox_max(1) && pos(2) >= bbox_min(2) && pos(2) <= bbox_max(2);
          };
          if (!isInBox(nbr_pos)) {
            continue;
          }

          if (!map_server_->getTSDF()->isInBox(nbr_pos)) {
            continue;
          }

          if (map_server_->getOccupancyGrid()->getVoxel(nbr_pos).value ==
                  voxel_mapping::OccupancyType::OCCUPIED ||
              map_server_->getOccupancyGrid()->getVoxel(nbr_pos).value ==
                  voxel_mapping::OccupancyType::UNKNOWN) {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                    voxel_mapping::OccupancyType::OCCUPIED ||
                map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                    voxel_mapping::OccupancyType::UNKNOWN) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) {
            continue;
          }

          AstarNode::Ptr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == config_.allocate_num_) {
              ROS_WARN("[AStar] Run out of node pool. Duration: %f",
                       (ros::Time::now() - t1).toSec());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + config_.lambda_heuristic_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

// TODO: modify this function to search in unknown space with penalty
int Astar::searchUnknown(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) {
  AstarNode::Ptr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = config_.lambda_heuristic_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) && cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    if (abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 &&
        abs(cur_node->index(2) - end_index(2)) <= 1) {
      bool safe = true;
      Vector3d dir = end_pt - cur_node->position;
      double len = dir.norm();
      dir.normalize();
      for (double l = 0.1; l < len; l += 0.1) {
        Vector3d ckpt = cur_node->position + l * dir;
        if (map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
            voxel_mapping::OccupancyType::OCCUPIED) {
          safe = false;
          break;
        }
      }
      if (safe) {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > config_.max_search_time_) {
      // ROS_WARN("[AStar] Early terminated, duration: %f", (ros::Time::now() - t1).toSec());
      // early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -config_.resolution_ + config_.epsilon_;
         dx <= config_.resolution_ + config_.epsilon_; dx += config_.resolution_)
      for (double dy = -config_.resolution_ + config_.epsilon_;
           dy <= config_.resolution_ + config_.epsilon_; dy += config_.resolution_)
        for (double dz = -config_.resolution_ + config_.epsilon_;
             dz <= config_.resolution_ + config_.epsilon_; dz += config_.resolution_) {
          step << dx, dy, dz;
          if (step.norm() < config_.epsilon_)
            continue;
          nbr_pos = cur_pos + step;
          // Check safety
          if (!map_server_->getTSDF()->isInBox(nbr_pos)) {
            continue;
          }

          if (map_server_->getOccupancyGrid()->getVoxel(nbr_pos).value ==
              voxel_mapping::OccupancyType::OCCUPIED) {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                voxel_mapping::OccupancyType::OCCUPIED) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) {
            continue;
          }

          AstarNode::Ptr neighbor;
          double tmp_g_score;
          // if (map_server_->getOccupancyGrid()->getVoxel(nbr_pos).value ==
          //     voxel_mapping::OccupancyType::UNKNOWN)
          //   tmp_g_score = step.norm() * unknown_penalty_factor_ + cur_node->g_score;
          // else
          tmp_g_score = step.norm() + cur_node->g_score;

          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == config_.allocate_num_) {
              ROS_WARN("[AStar] Run out of node pool. Duration: %f",
                       (ros::Time::now() - t1).toSec());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + config_.lambda_heuristic_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

int Astar::searchUnknownBBox(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
                             const Eigen::Vector3d &bbox_min, const Eigen::Vector3d &bbox_max) {
  AstarNode::Ptr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = config_.lambda_heuristic_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) && cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    if (abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 &&
        abs(cur_node->index(2) - end_index(2)) <= 1) {
      bool safe = true;
      Vector3d dir = end_pt - cur_node->position;
      double len = dir.norm();
      dir.normalize();
      for (double l = 0.1; l < len; l += 0.1) {
        Vector3d ckpt = cur_node->position + l * dir;
        if (map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                voxel_mapping::OccupancyType::OCCUPIED ||
            map_server_->getESDF()->getVoxel(ckpt).value < 0.5) {
          safe = false;
          break;
        }
      }
      if (safe) {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > config_.max_search_time_) {
      // ROS_WARN("[AStar] Early terminated, duration: %f", (ros::Time::now() - t1).toSec());
      // early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -config_.resolution_ + config_.epsilon_;
         dx <= config_.resolution_ + config_.epsilon_; dx += config_.resolution_)
      for (double dy = -config_.resolution_ + config_.epsilon_;
           dy <= config_.resolution_ + config_.epsilon_; dy += config_.resolution_)
        for (double dz = -config_.resolution_ + config_.epsilon_;
             dz <= config_.resolution_ + config_.epsilon_; dz += config_.resolution_) {
          step << dx, dy, dz;
          if (step.norm() < config_.epsilon_)
            continue;
          nbr_pos = cur_pos + step;
          // Check safety
          auto isInBox = [&](const Eigen::Vector3d &pos) {
            return pos(0) >= bbox_min(0) && pos(0) <= bbox_max(0) && pos(1) >= bbox_min(1) &&
                   pos(1) <= bbox_max(1) && pos(2) >= bbox_min(2) && pos(2) <= bbox_max(2);
          };
          if (!isInBox(nbr_pos)) {
            continue;
          }

          if (map_server_->getOccupancyGrid()->getVoxel(nbr_pos).value ==
                  voxel_mapping::OccupancyType::OCCUPIED ||
              map_server_->getESDF()->getVoxel(nbr_pos).value < 0.5) {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                    voxel_mapping::OccupancyType::OCCUPIED ||
                map_server_->getESDF()->getVoxel(ckpt).value < 0.5) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) {
            continue;
          }

          AstarNode::Ptr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == config_.allocate_num_) {
              ROS_WARN("[AStar] Run out of node pool. Duration: %f",
                       (ros::Time::now() - t1).toSec());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + config_.lambda_heuristic_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

int Astar::searchUnknownOnlyBBox(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
                                 const Eigen::Vector3d &bbox_min, const Eigen::Vector3d &bbox_max) {
  AstarNode::Ptr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = config_.lambda_heuristic_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) && cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    if (abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 &&
        abs(cur_node->index(2) - end_index(2)) <= 1) {
      bool safe = true;
      Vector3d dir = end_pt - cur_node->position;
      double len = dir.norm();
      dir.normalize();
      for (double l = 0.1; l < len; l += 0.1) {
        Vector3d ckpt = cur_node->position + l * dir;
        if (map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                voxel_mapping::OccupancyType::OCCUPIED ||
            map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                voxel_mapping::OccupancyType::FREE) {
          safe = false;
          break;
        }
      }
      if (safe) {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > config_.max_search_time_) {
      // ROS_WARN("[AStar] Early terminated, duration: %f", (ros::Time::now() - t1).toSec());
      // early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -config_.resolution_ + config_.epsilon_;
         dx <= config_.resolution_ + config_.epsilon_; dx += config_.resolution_)
      for (double dy = -config_.resolution_ + config_.epsilon_;
           dy <= config_.resolution_ + config_.epsilon_; dy += config_.resolution_)
        for (double dz = -config_.resolution_ + config_.epsilon_;
             dz <= config_.resolution_ + config_.epsilon_; dz += config_.resolution_) {
          step << dx, dy, dz;
          if (step.norm() < config_.epsilon_)
            continue;
          nbr_pos = cur_pos + step;
          // Check safety
          auto isInBox = [&](const Eigen::Vector3d &pos) {
            return pos(0) >= bbox_min(0) && pos(0) <= bbox_max(0) && pos(1) >= bbox_min(1) &&
                   pos(1) <= bbox_max(1) && pos(2) >= bbox_min(2) && pos(2) <= bbox_max(2);
          };
          if (!isInBox(nbr_pos)) {
            continue;
          }

          if (map_server_->getOccupancyGrid()->getVoxel(nbr_pos).value ==
                  voxel_mapping::OccupancyType::OCCUPIED ||
              map_server_->getOccupancyGrid()->getVoxel(nbr_pos).value ==
                  voxel_mapping::OccupancyType::FREE) {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                    voxel_mapping::OccupancyType::OCCUPIED ||
                map_server_->getOccupancyGrid()->getVoxel(ckpt).value ==
                    voxel_mapping::OccupancyType::FREE) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) {
            continue;
          }

          AstarNode::Ptr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == config_.allocate_num_) {
              ROS_WARN("[AStar] Run out of node pool. Duration: %f",
                       (ros::Time::now() - t1).toSec());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + config_.lambda_heuristic_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void Astar::reset() {
  open_set_map_.clear();
  close_set_map_.clear();
  path_nodes_.clear();

  std::priority_queue<AstarNode::Ptr, std::vector<AstarNode::Ptr>, AstarNodeComparator> empty_queue;
  open_set_.swap(empty_queue);
  for (int i = 0; i < (int)use_node_num_; i++) {
    path_node_pool_[i]->parent = NULL;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
}

double Astar::pathLength(const vector<Eigen::Vector3d> &path) {
  double length = 0.0;
  if ((int)path.size() < 2)
    return length;
  for (int i = 0; i < (int)path.size() - 1; ++i)
    length += (path[i + 1] - path[i]).norm();
  return length;
}

void Astar::backtrack(const AstarNode::Ptr &end_node, const Eigen::Vector3d &end) {
  path_nodes_.push_back(end);
  path_nodes_.push_back(end_node->position);
  AstarNode::Ptr cur_node = end_node;
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node->position);
  }
  reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> Astar::getPath() { return path_nodes_; }
double Astar::getResolution() { return config_.resolution_; }
double Astar::getMaxSearchTime() { return config_.max_search_time_; }

double Astar::getEuclHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2) {
  return config_.tie_breaker_ * (x2 - x1).norm();
}

double Astar::getManhHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  return config_.tie_breaker_ * (dx + dy + dz);
}

double Astar::getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  double h = 0.0;
  double diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < config_.epsilon_) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < config_.epsilon_) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < config_.epsilon_) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return config_.tie_breaker_ * h;
}

void Astar::posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) {
  idx = ((pt - config_.map_origin_) * config_.resolution_inverse_).array().floor().cast<int>();
}

void Astar::shortenPath(vector<Vector3d> &path) {
  if (path.empty()) {
    ROS_ERROR("[Astar] Empty path to shorten.");
    return;
  }

  if (path.size() == 1) {
    ROS_ERROR("[Astar] Single point path to shorten.");
    return;
  }

  // Shorten the tour, only critical intermediate points are reserved.
  // See Fig.8 from "Robust Real-time UAV Replanning Using Guided Gradient-based
  // Optimization and Topological Paths"
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = {path.front()};

  for (int i = 1; i < (int)path.size(); ++i) {
    // the end point
    if (i == (int)path.size() - 1) {
      if ((path[i] - short_tour.back()).norm() > dist_thresh) {
        int segment_num = std::floor((path[i] - short_tour.back()).norm() / dist_thresh);
        double segment_len = (path[i] - short_tour.back()).norm() / segment_num;
        Eigen::Vector3d dir = (path[i] - short_tour.back()).normalized();
        for (int j = 1; j <= segment_num; ++j)
          short_tour.push_back(short_tour.back() + segment_len * dir);
      } else {
        short_tour.push_back(path[i]);
      }
    } else {
      // Add waypoints to shorten path only to avoid collision
      bool occluded = false;
      ray_caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ray_caster_->nextId(idx) && ros::ok()) {
        if (map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::OCCUPIED ||
            map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::UNKNOWN) {
          // Occuluded by obstacles or unknown space,
          // short_tour.push_back(path[i]);
          occluded = true;
          break;
        }
      }

      if (occluded) {
        if ((path[i] - short_tour.back()).norm() > dist_thresh) {
          int segment_num = std::floor((path[i] - short_tour.back()).norm() / dist_thresh);
          double segment_len = (path[i] - short_tour.back()).norm() / segment_num;
          Eigen::Vector3d dir = (path[i] - short_tour.back()).normalized();
          for (int j = 1; j <= segment_num; ++j)
            short_tour.push_back(short_tour.back() + segment_len * dir);
        } else {
          short_tour.push_back(path[i]);
        }
      }
    }
  }

  // for (int i = 1; i < path.size(); ++i) {
  //   if ((path[i] - short_tour.back()).norm() > dist_thresh || i == path.size() - 1) {
  //     int segment_num = std::floor((path[i] - short_tour.back()).norm() / dist_thresh);
  //     if (segment_num == 0) // happens when i == path.size() - 1
  //       segment_num = 1;
  //     double segment_len = (path[i] - short_tour.back()).norm() / segment_num;
  //     Eigen::Vector3d dir = (path[i] - short_tour.back()).normalized();
  //     for (int j = 1; j <= segment_num; ++j)
  //       short_tour.push_back(short_tour.back() + segment_len * dir);
  //   } else {
  //     // Add waypoints to shorten path only to avoid collision
  //     ray_caster_->input(short_tour.back(), path[i + 1]);
  //     Eigen::Vector3i idx;
  //     while (ray_caster_->nextId(idx) && ros::ok()) {
  //       if (map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::OCCUPIED ||
  //           map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::UNKNOWN) {
  //         short_tour.push_back(path[i]);
  //         break;
  //       }
  //     }
  //   }
  // }

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));

  path = short_tour;
}

} // namespace fast_planner
