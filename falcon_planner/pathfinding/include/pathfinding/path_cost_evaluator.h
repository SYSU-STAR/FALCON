#ifndef _GRAPH_NODE_H_
#define _GRAPH_NODE_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <list>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>

#include "pathfinding/astar.h"
#include "raycast/raycast.h"

using Eigen::Vector3d;
using Eigen::Vector3i;
using std::cout;
using std::list;
using std::queue;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

class RayCaster;

using voxel_mapping::MapServer;

namespace fast_planner {
class Astar;
class PathCostEvaluator {
public:
  typedef shared_ptr<PathCostEvaluator> Ptr;

  PathCostEvaluator() {}
  ~PathCostEvaluator() {}

  static double computePathCost(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                const double &y2, const Vector3d &v1, const double &yd1,
                                const vector<Vector3d> &path, const double &len,
                                bool verbose = false);

  static double computeCost(const Vector3d &p1, const Vector3d &p2, const double &y1,
                            const double &y2, const Vector3d &v1, const double &yd1,
                            vector<Vector3d> &path, bool verbose = false);
  static double computeCostBBox(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                const double &y2, const Vector3d &v1, const double &yd1,
                                const Vector3d &bbox_min, const Vector3d &bbox_max,
                                vector<Vector3d> &path, bool verbose = false);
  static double computeCostUnknown(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                   const double &y2, const Vector3d &v1, const double &yd1,
                                   vector<Vector3d> &path);
  static double computeCostUnknownBBox(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                       const double &y2, const Vector3d &v1, const double &yd1,
                                       const Vector3d &bbox_min, const Vector3d &bbox_max,
                                       vector<Vector3d> &path);
  static double computeCostUnknownOnlyBBox(const Vector3d &p1, const Vector3d &p2, const double &y1,
                                           const double &y2, const Vector3d &v1, const double &yd1,
                                           const Vector3d &bbox_min, const Vector3d &bbox_max,
                                           vector<Vector3d> &path);

  // Coarse to fine path searching
  static double searchPath(const Vector3d &p1, const Vector3d &p2, vector<Vector3d> &path);
  static double searchPathBBox(const Vector3d &p1, const Vector3d &p2, const Vector3d &bbox_min,
                               const Vector3d &bbox_max, vector<Vector3d> &path);
  static double searchPathUnknown(const Vector3d &p1, const Vector3d &p2, vector<Vector3d> &path);
  static double searchPathUnknownBBox(const Vector3d &p1, const Vector3d &p2,
                                      const Vector3d &bbox_min, const Vector3d &bbox_max,
                                      vector<Vector3d> &path);
  static double searchPathUnknownOnlyBBox(const Vector3d &p1, const Vector3d &p2,
                                          const Vector3d &bbox_min, const Vector3d &bbox_max,
                                          vector<Vector3d> &path);

  // Parameters shared among nodes
  static double vm_, am_, yd_, ydd_;
  static Astar::Ptr astar_;
  static RayCaster::Ptr caster_;
  static MapServer::Ptr map_server_;
};
} // namespace fast_planner
#endif