#include "voxel_mapping/esdf.h"
#include "voxel_mapping/occupancy_grid.h"
#include "voxel_mapping/tsdf.h"

namespace voxel_mapping {
void ESDF::inputPointCloud(const PointCloudType &pointcloud) {
  std::cout << "ESDF cannot be constructed directly from pointcloud yet" << std::endl;
}

// Update local by TSDF updating bounding box
void ESDF::updateLocalESDF(const VoxelIndex &idx_min, const VoxelIndex &idx_max) {
  for (int x = idx_min[0]; x <= idx_max[0]; x++) {
    for (int y = idx_min[1]; y <= idx_max[1]; y++) {
      fillESDF(
          [&](int z) {
            return (occupancy_grid_->getVoxel(indexToAddress(VoxelIndex(x, y, z))).value ==
                    OccupancyType::OCCUPIED)
                       ? 0
                       : std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1_[indexToAddress(VoxelIndex(x, y, z))].value = val; },
          idx_min[2], idx_max[2], 2);
    }
  }
  for (int x = idx_min[0]; x <= idx_max[0]; x++) {
    for (int z = idx_min[2]; z <= idx_max[2]; z++) {
      fillESDF(
          [&](int y) { return tmp_buffer1_[indexToAddress(VoxelIndex(x, y, z))].value; },
          [&](int y, double val) { tmp_buffer2_[indexToAddress(VoxelIndex(x, y, z))].value = val; },
          idx_min[1], idx_max[1], 1);
    }
  }

  for (int y = idx_min[1]; y <= idx_max[1]; y++) {
    for (int z = idx_min[2]; z <= idx_max[2]; z++) {
      fillESDF([&](int x) { return tmp_buffer2_[indexToAddress(VoxelIndex(x, y, z))].value; },
               [&](int x, double val) {
                 double dist = map_config_.resolution_ * std::sqrt(val);
                 if (dist < map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value)
                   map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value = dist;
               },
               idx_min[0], idx_max[0], 0);
    }
  }
}

template <typename F_get_val, typename F_set_val>
void ESDF::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  // std::cout << "fillESDF: " << start << " " << end << " " << dim << std::endl;

  int v[map_config_.map_size_idx_(dim)];
  double z[map_config_.map_size_idx_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

double ESDF::getDistance(const Position &pos) {
  CHECK(isInMap(pos));

  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * map_config_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx = positionToIndex(pos_m);
  Eigen::Vector3d idx_pos = indexToPosition(idx);
  Eigen::Vector3d diff = (pos - idx_pos) * map_config_.resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  return dist;
}

double ESDF::getDistance(const VoxelIndex &idx) {
  CHECK(isInMap(idx)) << "Index: " << idx.transpose()
                      << ", Pos: " << indexToPosition(idx).transpose() << " is out of map range";

  return map_data_->data[indexToAddress(idx)].value;
}

void ESDF::getDistanceAndGradient(const Position &pos, double &distance,
                                  Eigen::Vector3d &gradient) {
  // Trilinear Interpolation
  // Find the start vertex of the interpolation cube, i.e. C000
  Eigen::Vector3d pos_m = pos - 0.5 * map_config_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i vertex_idx = positionToIndex(pos_m);
  Eigen::Vector3d vertex_pos = indexToPosition(vertex_idx);

  // Get the interpolation value
  Eigen::Vector3d diff = (pos - vertex_pos) * map_config_.resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = vertex_idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  distance = (1 - diff[2]) * v0 + diff[2] * v1;

  gradient[2] = v1 - v0;
  gradient[1] = (1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01);
  gradient[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  gradient[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  gradient[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  gradient[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  gradient.normalize();
}

void ESDF::getDistanceAndGradient(const VoxelIndex &idx, double &distance,
                                  Eigen::Vector3d &gradient) {
  CHECK(isInMap(idx));
  getDistanceAndGradient(indexToPosition(idx), distance, gradient);
}

void ESDF::saveMap(const string &filename) {
  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }

  for (size_t i = 0; i < map_data_->data.size(); ++i) {
    ofs << (FloatingPoint)map_data_->data[i].value << " ";
  }

  ofs.close();
}

void ESDF::loadMap(const string &filename) {
  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }
  for (size_t i = 0; i < map_data_->data.size(); ++i) {
    FloatingPoint value;
    ifs >> value;
    map_data_->data[i].value = value;
  }
  ifs.close();
}

void ESDF::loadMapFromPCL(const PointCloudType::Ptr cloud) {
  // Downsample pcl
  PointCloudType::Ptr cloud_filtered(new PointCloudType);
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*cloud_filtered);

  // KD-Tree
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  tree->setInputCloud(cloud_filtered);

  // ESDF
  for (int x = map_config_.box_min_idx_[0]; x < map_config_.box_max_idx_[0]; ++x) {
    for (int y = map_config_.box_min_idx_[1]; y < map_config_.box_max_idx_[1]; ++y) {
      for (int z = map_config_.box_min_idx_[2]; z < map_config_.box_max_idx_[2]; ++z) {
        Eigen::Vector3d pos = indexToPosition(VoxelIndex(x, y, z));
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if (tree->nearestKSearch(PointType(pos[0], pos[1], pos[2]), 1, pointIdxNKNSearch,
                                 pointNKNSquaredDistance) > 0) {
          map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value =
              std::sqrt(pointNKNSquaredDistance[0]);
        } else {
          std::cout << "x: " << x << ", y: " << y << ", z: " << z
                    << ", value: " << map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value
                    << std::endl;
          map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value =
              std::numeric_limits<double>::max();
        }

        // std::cout << "x: " << x << ", y: " << y << ", z: " << z
        //           << ", value: " << map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value
        //           << std::endl;
      }
    }
  }

  // saveMap("/home/eason/esdf.txt");
  // loadMap("/home/eason/esdf.txt");
}

void ESDF::loadMapFromPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_occ,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_free) {
  // Downsample pcl
  pcl::VoxelGrid<PointType> sor;

  PointCloudType::Ptr cloud_occ_filtered(new PointCloudType);
  sor.setInputCloud(cloud_occ);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*cloud_occ_filtered);

  PointCloudType::Ptr cloud_free_filtered(new PointCloudType);
  sor.setInputCloud(cloud_free);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*cloud_free_filtered);

  // KD-Tree
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  tree->setInputCloud(cloud_occ_filtered);

  // ESDF
  for (int x = map_config_.box_min_idx_[0]; x < map_config_.box_max_idx_[0]; ++x) {
    for (int y = map_config_.box_min_idx_[1]; y < map_config_.box_max_idx_[1]; ++y) {
      for (int z = map_config_.box_min_idx_[2]; z < map_config_.box_max_idx_[2]; ++z) {
        Eigen::Vector3d pos = indexToPosition(VoxelIndex(x, y, z));
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if (tree->nearestKSearch(PointType(pos[0], pos[1], pos[2]), 1, pointIdxNKNSearch,
                                 pointNKNSquaredDistance) > 0) {
          map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value =
              std::sqrt(pointNKNSquaredDistance[0]);
        } else {
          map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value =
              std::numeric_limits<double>::max();
        }
      }
    }
  }

  // ESDF for inner object
  tree->setInputCloud(cloud_free_filtered);
  for (int x = map_config_.box_min_idx_[0]; x < map_config_.box_max_idx_[0]; ++x) {
    for (int y = map_config_.box_min_idx_[1]; y < map_config_.box_max_idx_[1]; ++y) {
      for (int z = map_config_.box_min_idx_[2]; z < map_config_.box_max_idx_[2]; ++z) {
        if (map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value < 1e-4) {
          Eigen::Vector3d pos = indexToPosition(VoxelIndex(x, y, z));
          std::vector<int> pointIdxNKNSearch(1);
          std::vector<float> pointNKNSquaredDistance(1);
          if (tree->nearestKSearch(PointType(pos[0], pos[1], pos[2]), 1, pointIdxNKNSearch,
                                   pointNKNSquaredDistance) > 0) {
            double dist = -std::sqrt(pointNKNSquaredDistance[0]);
            if (dist < map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value) {
              map_data_->data[indexToAddress(VoxelIndex(x, y, z))].value = dist;
            }
          }
        }
      }
    }
  }
}

} // namespace voxel_mapping