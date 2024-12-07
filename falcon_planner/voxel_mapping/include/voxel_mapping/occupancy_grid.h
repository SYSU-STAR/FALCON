#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <fstream>
#include <iostream>
#include <string>

#include "voxel_mapping/map_base.h"
#include "voxel_mapping/voxel.h"

using std::string;

namespace voxel_mapping {
class TSDF;

class OccupancyGrid : public MapBase<OccupancyVoxel> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<OccupancyGrid> Ptr;
  typedef shared_ptr<const OccupancyGrid> ConstPtr;

  struct Config {
    enum class MODE { GEN_TSDF, GEN_PCL };

    double sdf_cutoff_dist_;
    MODE mode_;
  };

  OccupancyGrid(){};
  ~OccupancyGrid(){};

  void inputPointCloud(const PointCloudType &pointcloud);

  void updateOccupancyVoxel(const VoxelAddress &addr);

  void setTSDF(const shared_ptr<TSDF> &tsdf) { tsdf_ = tsdf; }

  bool setOccupancy(const VoxelAddress &addr, const OccupancyType &type);

  void getOccupancyPointcloud(PointCloudType::Ptr pointcloud,
                              const OccupancyType &type = OccupancyType::OCCUPIED);
  void publishPointcloud(PointCloudType pcd);

  void saveMap(const string &filename);
  void saveMapToPCD(const string &filename);
  void loadMap(const string &filename);
  void loadMapFromPCL(const PointCloudType::Ptr &cloud,
                      const OccupancyType &type = OccupancyType::OCCUPIED,
                      const Eigen::Vector3i xyz_inflate = Eigen::Vector3i::Zero(),
                      const bool setUnknownFree = false);
  void loadMapFromPCD(const string &filename, const OccupancyType &type = OccupancyType::OCCUPIED);
  void loadMapFromPLY(const string &filename, const OccupancyType &type = OccupancyType::OCCUPIED);

  Config config_;

private:
  shared_ptr<TSDF> tsdf_;
};
} // namespace voxel_mapping

#endif // ESDF_H