#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Eigen>
#include <glog/logging.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "exploration_preprocessing/connectivity_graph.h"
#include "pathfinding/path_cost_evaluator.h"
#include "exploration_types.h"
#include "tic_toc.h"
#include "voxel_mapping/map_server.h"

// Grid cell: A single cell in the uniform grid
// Uniform Grid: A grid (one layer in the hierarchical grid) with uniform size cells
// Hierarchical Grid: A grid data structure including multiple layers of uniform grids

namespace fast_planner {
// Class definition for a single grid cell
struct GridCell {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class STATE { ACTIVE, EXPLORED, UNKNWON };
  // The cell ID is also its address in the uniform grid vector
  // The address is calculated as: id = x + y * num_cells_x + z * num_cells_x * num_cells_y
  int id_;
  STATE state_ = STATE::ACTIVE;

  Position center_, center_free_;

  // centers_free_: CCL centers of free voxels for each free subregions in the cell
  // centers_free_active_: centers_free_ with frontier vps in the cell, used for global CP planning
  // centers_unknown_: CCL centers of unknown voxels for each unknown subregions  in the cell
  vector<Position> centers_free_, centers_free_active_, centers_unknown_, centers_unknown_active_;
  vector<int> centers_free_active_idx_, centers_unknown_active_idx_;

  Position bbox_min_, bbox_max_;
  VoxelIndex bbox_min_idx_, bbox_max_idx_;
  std::vector<int> nearby_cells_ids_;

  int unknown_num_, free_num_;
  int frontier_num_;

  // Frontier information for the cell in uniform grid
  std::vector<int> frontier_ids_;
  std::vector<Position> frontier_viewpoints_;
  std::vector<double> frontier_yaws_;

  // Frontier information for centers_free_active_
  // a cell in uniform grid may have multiple free subregions
  // each subregion has its own CCL centers (free and unknown)
  // each free CCL center has its own frontier information (might be empty or multiple)
  // frontier_ids_mc_.size() == centers_free_active_.size()
  std::vector<std::vector<int>> frontier_ids_mc_;
  std::vector<std::vector<Position>> frontier_viewpoints_mc_;
  std::vector<std::vector<double>> frontier_yaws_mc_;

  // Connectivity matrixs for current cell CCL centers and nearby cells' CCL centers
  // matrix size is (centers_free_.size(), nearby_cell_centers_free.size())
  // map: nearby cell id -> connectivity matrix
  std::map<int, std::vector<std::vector<bool>>> connectivity_matrixs_;

  // Visualization vertices of each side of the grid cell
  std::vector<Position> vertices1_, vertices2_;

  void print() {
    std::cout << "-------------------" << std::endl;
    std::cout << "Grid cell id: " << id_ << std::endl;
    std::cout << "Center: " << center_.transpose() << std::endl;
    std::cout << "Bbox min: " << bbox_min_.transpose() << std::endl;
    std::cout << "Bbox max: " << bbox_max_.transpose() << std::endl;
    std::cout << "Unknown num: " << unknown_num_ << std::endl;
    std::cout << "Frontier num: " << frontier_num_ << std::endl;
    std::cout << "Frontier ids: ";
    for (auto it = frontier_ids_.begin(); it != frontier_ids_.end(); ++it) {
      std::cout << *it << " ";
    }
    std::cout << std::endl;
    std::cout << "-------------------" << std::endl;
  }
};

// Class definition for a uniform grid (one layer in the hierarchical grid)
class UniformGrid {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<UniformGrid> Ptr;
  typedef std::shared_ptr<const UniformGrid> ConstPtr;

  struct Config {
    int level_;
    int num_cells_x_, num_cells_y_, num_cells_z_, num_cells_;
    Eigen::Vector3i num_voxels_per_cell_; // Number of voxels in each cell, x, y, z num may differ
    Eigen::Vector3d cell_size_;           // Size of each grid cell
    Position bbox_min_, bbox_max_;        // Bounding box of the grid
    double map_resolution_;               // Resolution of the map (m/voxel)
    bool use_2d_grids_;                   // True if z partitioning is ignored
    bool verbose_;                        // True if print out debug information

    // Cost matrix calculation parameters
    double consistency_gain_;

    int min_unknown_num_; // Minimum number of unknown voxels in an active cell

    double unknown_penalty_factor_;

    double hybrid_search_radius_;

    int ccl_step_;

    double epsilon_ = 1e-4;

    void print() {
      std::cout << "-------------------" << std::endl;
      std::cout << "Uniform grid level: " << level_ << std::endl;
      std::cout << "Number of cells: " << num_cells_ << std::endl;
      std::cout << "Number of cells in x: " << num_cells_x_ << std::endl;
      std::cout << "Number of cells in y: " << num_cells_y_ << std::endl;
      std::cout << "Number of cells in z: " << num_cells_z_ << std::endl;
      std::cout << "Number of voxels per cell: " << num_voxels_per_cell_.transpose() << std::endl;
      std::cout << "Cell size: " << cell_size_.transpose() << std::endl;
      std::cout << "Bounding box min: " << bbox_min_.transpose() << std::endl;
      std::cout << "Bounding box max: " << bbox_max_.transpose() << std::endl;
      std::cout << "Map resolution: " << map_resolution_ << std::endl;
      std::cout << "Use 2d grids: " << use_2d_grids_ << std::endl;
      std::cout << "Verbose: " << verbose_ << std::endl;
      std::cout << "Consistency gain: " << consistency_gain_ << std::endl;
      std::cout << "Minimum unknown num: " << min_unknown_num_ << std::endl;
      std::cout << "Unknown penalty factor: " << unknown_penalty_factor_ << std::endl;
      std::cout << "Epsilon: " << epsilon_ << std::endl;
      std::cout << "-------------------" << std::endl;
    }
  };

  enum class CENTER_TYPE { UNKNOWN, FREE };

  UniformGrid();
  ~UniformGrid();

  UniformGrid(ros::NodeHandle &nh, const Config &config, voxel_mapping::MapServer::Ptr &map_server);

  // Get the grid cell id of a given position
  int positionToGridCellId(const Position &pos);
  void positionToGridCellId(const Position &pos, int &id);
  void positionToGridCellCenterId(const Position &pos, int &cell_id, int &center_id);

  void getGridCellIdsInBBox(const Position &bbox_min, const Position &bbox_max,
                            std::vector<int> &cell_ids);

  // Grid update operations
  // Update frontier information in each cell
  void inputFrontiers(const std::vector<Position> &frontier_viewpoints,
                      const std::vector<double> &frontier_yaws);
  // Update grids cell information from voxel map
  void updateGridsFromVoxelMap(const Position &update_bbox_min, const Position &update_bbox_max);
  void updateGridsFrontierInfo(const Position &update_bbox_min, const Position &update_bbox_max);
  Position rectifyCellCenter(const int &cell_id, const Position &center, const Position &bbox_min,
                             const Position &bbox_max);
  Position rectifyCellCenter2(const int &cell_id, const Position &center, const Position &bbox_min,
                              const Position &bbox_max);
  void getCCLCenters(const int &cell_id, std::vector<Position> &centers_free,
                     std::vector<Position> &centers_unknown);
  void getCCLCenters2D(const int &cell_id, std::vector<Position> &centers_free,
                       std::vector<Position> &centers_unknown);

  // Cost matrix calculation for global planning
  double getNormalCost(const Position &pos1, const Position &pos2);
  double getXBiasedCost(const Position &pos1, const Position &pos2);
  double getAStarCost(const Position &pos1, const Position &pos2, const Eigen::Vector3d &vel);
  double getAStarCostUnknown(const Position &pos1, const Position &pos2,
                             const Eigen::Vector3d &vel);
  double getAStarCostYaw(const Position &pos1, const Position &pos2, const Eigen::Vector3d &vel,
                         const double &yaw1, const double &yaw2);
  double getAStarCostBBox(const Position &pos1, const Position &pos2, const Position &bbox_min,
                          const Position &bbox_max);
  double getAStarCostBBox(const Position &pos1, const Position &pos2, const Position &bbox_min,
                          const Position &bbox_max, std::vector<Position> &path);
  double getAStarCostBBoxUnknown(const Position &pos1, const Position &pos2,
                                 const Position &bbox_min, const Position &bbox_max);
  double getAStarCostBBoxUnknown(const Position &pos1, const Position &pos2,
                                 const Position &bbox_min, const Position &bbox_max,
                                 std::vector<Position> &path);
  double getAStarCostBBoxUnknownOnly(const Position &pos1, const Position &pos2,
                                     const Position &bbox_min, const Position &bbox_max);
  double getAStarCostBBoxUnknownOnly(const Position &pos1, const Position &pos2,
                                     const Position &bbox_min, const Position &bbox_max,
                                     std::vector<Position> &path);
  double getAStarCostHGridGraph(const Position &pos1, const Position &pos2);
  void calculateCostMatrixSingleThread(
      const Position &cur_pos, const Eigen::Vector3d &cur_vel, Eigen::MatrixXd &cost_matrix,
      std::map<int, std::pair<int, int>> &cost_mat_id_to_cell_center_id);
  void
  calculateCostMatrixMultiThread(const Position &cur_pos, const Eigen::Vector3d &cur_vel,
                                 const int &thread_num, Eigen::MatrixXd &cost_matrix,
                                 std::map<int, std::pair<int, int>> &cost_mat_id_to_cell_center_id);

  // Get functions to return parameters
  GridCell::STATE getCellState(const int &id) { return uniform_grid_[id].state_; }
  int getNumCells() { return config_.num_cells_; }
  void getCellBBox(const int &id, Position &bbox_min, Position &bbox_max) {
    bbox_min = uniform_grid_[id].bbox_min_;
    bbox_max = uniform_grid_[id].bbox_max_;
  }
  void getCellBBoxIndex(const int &id, VoxelIndex &bbox_min_idx, VoxelIndex &bbox_max_idx) {
    bbox_min_idx = uniform_grid_[id].bbox_min_idx_;
    bbox_max_idx = uniform_grid_[id].bbox_max_idx_;
  }
  Position getCellCenter(const int &id) { return uniform_grid_[id].center_; }
  void getCellCenter(const int &id, Position &center) { center = uniform_grid_[id].center_; }
  Position getCellCenters(const int &cell_id, const int &center_id) {
    const GridCell &cell = uniform_grid_[cell_id];
    if (center_id < cell.centers_free_active_.size()) {
      return cell.centers_free_active_[center_id];
    } else {
      return cell.centers_unknown_[center_id - cell.centers_free_active_.size()];
    }
  }
  void getCellCenters(const int &cell_id, const int &center_id, Position &center) {
    const GridCell &cell = uniform_grid_[cell_id];
    if (center_id < cell.centers_free_active_.size()) {
      center = cell.centers_free_active_[center_id];
    } else {
      center = cell.centers_unknown_active_[center_id - cell.centers_free_active_.size()];
    }
  }
  void getCellCenterType(const int &cell_id, const int &center_id, CENTER_TYPE &type) {
    if (center_id < 0) {
      type = CENTER_TYPE::UNKNOWN;
      return;
    }

    const GridCell &cell = uniform_grid_[cell_id];
    if (center_id < cell.centers_free_active_.size()) {
      type = CENTER_TYPE::FREE;
    } else {
      type = CENTER_TYPE::UNKNOWN;
    }
  }
  void getCellCentersWithType(const int &cell_id, const int &center_id, Position &center,
                              CENTER_TYPE &type) {
    const GridCell &cell = uniform_grid_[cell_id];
    if (center_id < cell.centers_free_active_.size()) {
      center = cell.centers_free_active_[center_id];
      type = CENTER_TYPE::FREE;
    } else {
      center = cell.centers_unknown_[center_id - cell.centers_free_active_.size()];
      type = CENTER_TYPE::UNKNOWN;
    }
  }
  void getActiveCellIds(std::vector<int> &active_cell_ids) { active_cell_ids = active_cell_ids_; }
  void getCellFrontiers(const int &id, std::vector<int> &frontier_ids,
                        std::vector<Position> &frontier_vps) {
    frontier_ids = uniform_grid_[id].frontier_ids_;
    frontier_vps = uniform_grid_[id].frontier_viewpoints_;
  }
  void getCellCenterFrontiers(const int &cell_id, const int &center_id,
                              std::vector<int> &frontier_ids, std::vector<Position> &frontier_vps) {
    frontier_ids = uniform_grid_[cell_id].frontier_ids_mc_[center_id];
    frontier_vps = uniform_grid_[cell_id].frontier_viewpoints_mc_[center_id];
  }
  void getNearbyCellIds(const int &id, std::vector<int> &nearby_cell_ids) {
    nearby_cell_ids = uniform_grid_[id].nearby_cells_ids_;
  }
  // Eigen::MatrixXd getCostMatrix() { return cost_matrix_; }
  const std::vector<std::vector<double>> &getCostMatrix() { return cost_matrix_; }
  void getSpaceDecompTime(double &time) { time = space_decomp_time_; }
  void getConnectivityGraphTime(double &time) { time = connectivity_graph_time_; }

  // Visualization: Get each pair of points for the outline of the grid
  void getVisualizationLineLists(std::vector<Position> &list1, std::vector<Position> &list2);
  void getVisualizationLineLists2D(std::vector<Position> &list1, std::vector<Position> &list2,
                                   double height = 1.0);
  void getVisualizationConnectivityLineLists(std::vector<Position> &list1,
                                             std::vector<Position> &list2);
  void getVisualizationIdText(std::vector<std::string> &cell_ids,
                              std::vector<Position> &cell_centers,
                              std::vector<std::vector<Position>> &cell_centers_free,
                              std::vector<std::vector<Position>> &cell_centers_unknown);

  friend class HierarchicalGrid;

private:
  Config config_;
  voxel_mapping::MapServer::Ptr map_server_;

  std::vector<GridCell> uniform_grid_;
  std::vector<int> active_cell_ids_;
  std::unordered_set<int> explored_cell_ids_;
  std::vector<int> cell_ids_need_update_frontier_;

  // Cost matrix for all cells
  std::vector<std::vector<double>> cost_matrix_;

  // CCL Voxels
  // std::set<int>: voxels address in one CCL zone, size = num of voxels (stepped) in this CCL zone
  // std::vector<std::set<int>>: CCL zones in one cell, size = num of CCL zones in this cell
  // std::vector<std::vector<std::set<int>>>: CCL zones in all cells, size = uniform_grid_.size()
  std::vector<std::vector<std::unordered_set<int>>> ccl_voxels_addr_;
  std::vector<std::vector<Eigen::Vector3d>> ccl_voxels_color_;
  std::vector<std::vector<std::pair<int, int>>> ccl_free_unknown_states_and_centers_idx_;

  // Connectivity graph
  ConnectivityGraph::Ptr connectivity_graph_;

  double space_decomp_time_, connectivity_graph_time_;
};

// Class definition for a hierarchical grid including multiple layers of uniform grids
class HierarchicalGrid {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config {
    int num_levels_;                // Number of levels in the hierarchical grid
    double cell_size_max_;          // Size of the largest grid (level 0)
    int cell_size_z_partition_num_; // Number of z partitioning for each grid
    bool use_2d_grids_;             // True if z partitioning is ignored
    bool verbose_;                  // True if print out debug information

    double min_unknown_num_scale_; // Scale of minimum unknown num w.r.t. cell numbers

    double unknown_penalty_factor_; // Penalty factor for unknown cells in astar cost calculation
    double hybrid_search_radius_;   // Radiusc cutoff for hybrid search
    int ccl_step_;                  // Step size for CCL
  };

  HierarchicalGrid();
  ~HierarchicalGrid();

  HierarchicalGrid(ros::NodeHandle &nh, voxel_mapping::MapServer::Ptr &map_server);

  // Update grid information from frontier
  void inputFrontiers(const std::vector<Position> &frontier_viewpoints,
                      const std::vector<double> &frontier_yaws);
  void updateHierarchicalGridFromVoxelMap(const Position &update_bbox_min,
                                          const Position &update_bbox_max);
  void updateHierarchicalGridFrontierInfo(const Position &update_bbox_min,
                                          const Position &update_bbox_max);

  // Cost matrix calculation for global planning
  void calculateCostMatrix2(const Position &cur_pos, const Eigen::Vector3d &cur_vel,
                            const double &cur_yaw, const std::vector<Position> &last_path,
                            Eigen::MatrixXd &cost_matrix,
                            std::map<int, std::pair<int, int>> &cost_mat_id_to_cell_center_id);

  // Get function to return parameters
  int getNumLevels() { return config_.num_levels_; }
  int getLayerNumCells(const int &level) { return uniform_grids_[level].getNumCells(); }
  int getLayerCellId(const int &level, const Position &pos) {
    int id = uniform_grids_[level].positionToGridCellId(pos);
    if (id < 0 || id >= uniform_grids_[level].getNumCells()) {
      return -1;
    }
    return id;
  }
  GridCell::STATE getLayerCellState(const int &level, const int &id) {
    return uniform_grids_[level].getCellState(id);
  }
  Position getLayerCellCenter(const int &level, const int &id) {
    return uniform_grids_[level].getCellCenter(id);
  }
  void getLayerCellCenter(const int &level, const int &id, Position &center) {
    uniform_grids_[level].getCellCenter(id, center);
  }
  Position getLayerCellCenters(const int &level, const int &cell_id, const int &center_id) {
    return uniform_grids_[level].getCellCenters(cell_id, center_id);
  }
  void getLayerCellCenters(const int &level, const int &cell_id, const int &center_id,
                           Position &center) {
    uniform_grids_[level].getCellCenters(cell_id, center_id, center);
  }
  void getLayerCellCenterType(const int &level, const int &cell_id, const int &center_id,
                              UniformGrid::CENTER_TYPE &center_type) {
    uniform_grids_[level].getCellCenterType(cell_id, center_id, center_type);
  }
  void getLayerActiveCellIds(const int &level, std::vector<int> &active_cell_ids) {
    uniform_grids_[level].getActiveCellIds(active_cell_ids);
  }
  void getLayerCellFrontiers(const int &level, const int &cell_id, std::vector<int> &frontier_ids,
                             std::vector<Position> &frontier_vps) {
    uniform_grids_[level].getCellFrontiers(cell_id, frontier_ids, frontier_vps);
  }
  void getLayerCellCenterFrontiers(const int &level, const int &cell_id, const int &center_id,
                                   std::vector<int> &frontier_ids,
                                   std::vector<Position> &frontier_vps) {
    uniform_grids_[level].getCellCenterFrontiers(cell_id, center_id, frontier_ids, frontier_vps);
  }
  void getLayerPositionCellCenterId(const int &level, const Position &pos, int &cell_id,
                                    int &center_id) {
    uniform_grids_[level].positionToGridCellCenterId(pos, cell_id, center_id);
  }
  void getLayerSpaceDecompTime(const int &level, double &time) {
    uniform_grids_[level].getSpaceDecompTime(time);
  }
  void getLayerConnectivityGraphTime(const int &level, double &time) {
    uniform_grids_[level].getConnectivityGraphTime(time);
  }
  void getLayerCellHeights(const int &level, std::vector<double> &cell_heights) {
    cell_heights.clear();
    Position bbox_min, bbox_max;
    map_server_->getBox(bbox_min, bbox_max);

    if (config_.use_2d_grids_) {
      cell_heights.push_back(bbox_max.z());
    } else {
      for (int i = 0; i < config_.cell_size_z_partition_num_; ++i) {
        cell_heights.push_back(bbox_min[2] + (bbox_max[2] - bbox_min[2]) /
                                                 config_.cell_size_z_partition_num_ * (i + 1));
      }
    }
  }

  // Visualization
  // Visulize one layer grid with its outlines and cell ids at each cell's center
  void publishGridsLayer(const int &level = 0);
  void publishGridsConnectivityLayer(const int &level = 0);
  void publishGridsConnectivityGraphLayer(const int &level = 0);
  void publishGridsCostMatrix(const int &level = 0);
  void publishActiveFreeAndUnknownCenters(const int &level = 0);

private:
  Config config_;

  // Levels of grids
  std::vector<UniformGrid> uniform_grids_;

  // Cost matrix of all cells (only for adjacent cells)
  Eigen::MatrixXd cost_matrix_;

  // Voxel mapping pointer shared from planner
  voxel_mapping::MapServer::Ptr map_server_;

  // Visulization publisher
  std::unordered_map<std::string, ros::Publisher> visulization_pubs_;
};
} // namespace fast_planner