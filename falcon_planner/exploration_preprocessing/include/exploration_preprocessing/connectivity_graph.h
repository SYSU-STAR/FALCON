#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "voxel_mapping/map_server.h"

namespace fast_planner {

struct ConnectivityEdge {
  typedef std::shared_ptr<ConnectivityEdge> Ptr;
  typedef std::shared_ptr<const ConnectivityEdge> ConstPtr;

  enum class TYPE { UNKNOWN, FREE, PORTAL };

  ConnectivityEdge() {}
  ~ConnectivityEdge() {}

  TYPE type_;
  int id1_, id2_;
  double cost_;

  std::vector<Position> path_;
};

struct ConnectivityNode {
  typedef std::shared_ptr<ConnectivityNode> Ptr;
  typedef std::shared_ptr<const ConnectivityNode> ConstPtr;

  enum class TYPE { UNKNOWN, FREE };

  ConnectivityNode() {}
  ConnectivityNode(const int &id, const Position &pos, const TYPE &type)
      : id_(id), pos_(pos), type_(type) {}
  ~ConnectivityNode() {}

  void addNeighbor(const int &neighbor_id, const double &cost, const ConnectivityEdge::TYPE &type) {
    ConnectivityEdge edge;
    edge.id1_ = id_;
    edge.id2_ = neighbor_id;
    edge.cost_ = cost;
    edge.type_ = type;

    // remove old edge
    for (auto it = neighbors_.begin(); it != neighbors_.end(); ++it) {
      if (it->id2_ == neighbor_id) {
        neighbors_.erase(it);
        break;
      }
    }

    neighbors_.push_back(edge);
  }
  void addNeighborWithPath(const int &neighbor_id, const double &cost,
                           const ConnectivityEdge::TYPE &type, const std::vector<Position> &path) {
    ConnectivityEdge edge;
    edge.id1_ = id_;
    edge.id2_ = neighbor_id;
    edge.cost_ = cost;
    edge.type_ = type;
    edge.path_ = path;

    // remove old edge
    for (auto it = neighbors_.begin(); it != neighbors_.end(); ++it) {
      if (it->id2_ == neighbor_id) {
        neighbors_.erase(it);
        break;
      }
    }

    neighbors_.push_back(edge);
  }
  void removeNeighbor(const int &neighbor_id) {
    for (auto it = neighbors_.begin(); it != neighbors_.end(); ++it) {
      if (it->id2_ == neighbor_id) {
        neighbors_.erase(it);
        break;
      }
    }
  }
  void clearNeighbors() { neighbors_.clear(); }

  // We assume a maximum number of 10 centers in one cell (which is enough for a camera)
  // Exceeding this number will cause an CHECK error in hierarchical_graph.cpp
  // id = cell_id * 10 + center_id
  TYPE type_;
  int id_;
  Position pos_; // actual position of the node

  std::vector<ConnectivityEdge> neighbors_; // id, cost (distance)
};

class ConnectivityGraph {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<ConnectivityGraph> Ptr;
  typedef std::shared_ptr<const ConnectivityGraph> ConstPtr;

  ConnectivityGraph() {}
  ~ConnectivityGraph() {}

  ConnectivityGraph(ros::NodeHandle &nh) {}

  void addNode(ConnectivityNode::Ptr node);
  void removeNode(const int &id);
  void clearNodes();
  ConnectivityNode::Ptr getNode(const int &id);
  void getNodeNum(int &num);
  int getNodeNum();
  void getNodePositions(std::vector<Position> &node_positions);
  void getNodePositionsWithIDs(std::vector<Position> &node_positions, std::vector<int> &node_ids);

  double searchConnectivityGraphBFS(const int &id1, const int &id2, std::vector<int> &path);
  double searchConnectivityGraphAStar(const int &id1, const int &id2, std::vector<int> &path);
  void getFullConnectivityGraph(std::vector<std::pair<Position, Position>> &edges,
                                std::vector<ConnectivityEdge::TYPE> &edges_types,
                                std::vector<double> &edges_costs);
  void getFullConnectivityGraphPath(std::vector<std::vector<Position>> &paths,
                                    std::vector<ConnectivityEdge::TYPE> &edges_types);

  void findDisconnectedNodes(std::set<int> &disconnected_nodes);

  bool saveConnectivityGraph(const std::string &file_path);
  bool loadConnectivityGraph(const std::string &file_path);

private:
  std::unordered_map<int, ConnectivityNode::Ptr> nodes_;

  double max_search_time_ = 1e-4;
};
} // namespace fast_planner