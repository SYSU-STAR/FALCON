#include "voxel_mapping/occupancy_grid.h"
#include "voxel_mapping/tsdf.h"

namespace voxel_mapping {
void OccupancyGrid::inputPointCloud(const PointCloudType &pointcloud) {
  assert(false); // Not implemented
}

void OccupancyGrid::updateOccupancyVoxel(const VoxelAddress &addr) {
  if (tsdf_->getVoxel(addr).weight < 1e-5) {
    map_data_->data[addr].value = OccupancyType::UNKNOWN;
    return;
  }

  if (tsdf_->getVoxel(addr).value < config_.sdf_cutoff_dist_) {
    map_data_->data[addr].value = OccupancyType::OCCUPIED;
  } else {
    map_data_->data[addr].value = OccupancyType::FREE;
  }
}

bool OccupancyGrid::setOccupancy(const VoxelAddress &addr, const OccupancyType &type) {
  if (addr < 0 || addr >= (int)map_data_->data.size()) {
    return false;
  }
  map_data_->data[addr].value = type;
  return true;
}

void OccupancyGrid::getOccupancyPointcloud(const PointCloudType::Ptr pointcloud,
                                           const OccupancyType &type) {
  pointcloud->clear();

  for (size_t i = 0; i < map_data_->data.size(); ++i) {
    if (map_data_->data[i].value == type) {
      Position pos = addressToPosition(i);
      PointType p;
      p.x = pos.x();
      p.y = pos.y();
      p.z = pos.z();
      pointcloud->points.push_back(p);
    }
  }
}

void OccupancyGrid::saveMap(const string &filename) {
  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }

  for (size_t i = 0; i < map_data_->data.size(); ++i) {
    ofs << (int)map_data_->data[i].value << " ";
  }

  ofs.close();
}

void OccupancyGrid::saveMapToPCD(const string &filename) {
  if (filename.find(".pcd") == string::npos) {
    std::cerr << "Invalid file name: " << filename << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_occupied(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_free(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < map_data_->data.size(); ++i) {
    if (map_data_->data[i].value == OccupancyType::OCCUPIED) {
      Position pos = indexToPosition(addressToIndex(i));
      pcl::PointXYZ p;
      p.x = pos.x();
      p.y = pos.y();
      p.z = pos.z();
      cloud_occupied->points.push_back(p);
    }
    if (map_data_->data[i].value == OccupancyType::FREE) {
      Position pos = indexToPosition(addressToIndex(i));
      pcl::PointXYZ p;
      p.x = pos.x();
      p.y = pos.y();
      p.z = pos.z();
      cloud_free->points.push_back(p);
    }
  }

  cloud_occupied->width = cloud_occupied->points.size();
  cloud_occupied->height = 1;
  cloud_free->width = cloud_free->points.size();
  cloud_free->height = 1;

  pcl::io::savePCDFileASCII(filename, *cloud_occupied);
  std::string free_filename = filename.substr(0, filename.size() - 4) + "_free.pcd";
  pcl::io::savePCDFileASCII(free_filename, *cloud_free);
}

void OccupancyGrid::loadMap(const string &filename) {
  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }
  for (size_t i = 0; i < map_data_->data.size(); ++i) {
    int value;
    ifs >> value;
    map_data_->data[i].value = (OccupancyType)value;
  }
  ifs.close();
}

void OccupancyGrid::loadMapFromPCL(const PointCloudType::Ptr &cloud, const OccupancyType &type,
                                   const Eigen::Vector3i xyz_inflate, const bool setUnknownFree) {
  // Set all voxels to free
  if (setUnknownFree) {
    for (size_t i = 0; i < map_data_->data.size(); ++i) {
      map_data_->data[i].value = OccupancyType::FREE;
    }
  }

  for (const PointType &p : cloud->points) {
    Position point(p.x, p.y, p.z);
    VoxelAddress voxel_addr = positionToAddress(point);
    if (voxel_addr < 0 || voxel_addr >= (int)map_data_->data.size()) {
      continue;
    }
    map_data_->data[voxel_addr].value = type;

    // Set surrouding voxels to occupied as original pcd is too sparse
    for (int dx = -xyz_inflate.x(); dx <= xyz_inflate.x(); ++dx) {
      for (int dy = -xyz_inflate.y(); dy <= xyz_inflate.y(); ++dy) {
        for (int dz = -xyz_inflate.z(); dz <= xyz_inflate.z(); ++dz) {
          VoxelIndex voxel_idx_surround = addressToIndex(voxel_addr) + VoxelIndex(dx, dy, dz);
          VoxelAddress voxel_addr_surround = indexToAddress(voxel_idx_surround);
          if (voxel_addr_surround > 0 && voxel_addr_surround < (int)map_data_->data.size())
            map_data_->data[voxel_addr_surround].value = type;
        }
      }
    }
  }
}

void OccupancyGrid::loadMapFromPCD(const string &filename, const OccupancyType &type) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
    std::cerr << "Failed to load pcd file: " << filename << std::endl;
    return;
  }
  loadMapFromPCL(cloud, type);
}

void OccupancyGrid::loadMapFromPLY(const string &filename, const OccupancyType &type) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1) {
    std::cerr << "Failed to load ply file: " << filename << std::endl;
    return;
  }
  loadMapFromPCL(cloud, type);
}

} // namespace voxel_mapping