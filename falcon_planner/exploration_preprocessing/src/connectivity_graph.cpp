#include "exploration_preprocessing/connectivity_graph.h"

namespace fast_planner {

void ConnectivityGraph::addNode(ConnectivityNode::Ptr node) {
  nodes_.insert(std::make_pair(node->id_, node));
}

void ConnectivityGraph::removeNode(const int &id) {
  // std::cout << "remove node " << id << std::endl;

  // // print all ids in nodes
  // std::cout << "all ids in nodes: ";
  // for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
  //   std::cout << it->first << " ";
  // }
  // std::cout << std::endl;

  if (nodes_.find(id) == nodes_.end()) {
    // std::cout << "node " << id << " does not exist" << std::endl;
    return;
  }

  ConnectivityNode::Ptr node = nodes_[id];
  // std::cout << "node id " << node->id_ << std::endl;

  // print all neighbors of node
  // std::cout << "neighbors of node " << id << ": ";
  // for (auto it = node->neighbors_.begin(); it != node->neighbors_.end(); ++it) {
  //   std::cout << it->id2_ << " ";
  // }
  // std::cout << std::endl;

  // iterate through all neighbors of node and remove node from their neighbors
  for (auto it = node->neighbors_.begin(); it != node->neighbors_.end(); ++it) {
    // std::cout << "remove node " << id << " from neighbors of node " << it->id2_ << std::endl;
    nodes_[it->id2_]->removeNeighbor(id);
  }

  nodes_.erase(id);
  // std::cout << "after remove node " << id << std::endl;
}

void ConnectivityGraph::clearNodes() { nodes_.clear(); }

ConnectivityNode::Ptr ConnectivityGraph::getNode(const int &id) {
  if (nodes_.find(id) == nodes_.end()) {
    return nullptr;
  }
  return nodes_[id];
}

void ConnectivityGraph::getNodeNum(int &num) { num = nodes_.size(); }

int ConnectivityGraph::getNodeNum() { return nodes_.size(); }

void ConnectivityGraph::getNodePositions(std::vector<Position> &node_positions) {
  node_positions.clear();
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    node_positions.push_back(it->second->pos_);
  }
}

void ConnectivityGraph::getNodePositionsWithIDs(std::vector<Position> &node_positions,
                                                std::vector<int> &node_ids) {
  node_positions.clear();
  node_ids.clear();
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    node_positions.push_back(it->second->pos_);
    node_ids.push_back(it->second->id_);
  }
}

double ConnectivityGraph::searchConnectivityGraphBFS(const int &id1, const int &id2,
                                                     std::vector<int> &path) {
  CHECK_NE(id1, id2) << "id1 == id2 in searchConnectivityGraphBFS";

  // std::cout << "searchConnectivityGraphBFS, id1: " << id1 << ", id2: " << id2 << std::endl;
  // ros::Time start_time = ros::Time::now();

  path.clear();
  // if (id1 == id2) {
  //   path.push_back(id1);
  //   return 0.0;
  // }

  if (nodes_.find(id1) == nodes_.end() || nodes_.find(id2) == nodes_.end()) {
    ROS_ERROR("[ConnectivityGraph] id1 %d or id2 %d does not exist", id1, id2);
    return 1000.0;
  }

  std::unordered_map<int, bool> visited_flags;
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    visited_flags.insert(std::make_pair(it->first, false));
  }

  std::queue<int> queue;
  std::unordered_map<int, int> parent_map;
  queue.push(id1);
  visited_flags[id1] = true;
  parent_map.insert(std::make_pair(id1, -1));

  bool found = false;
  while (!queue.empty()) {
    int id = queue.front();
    queue.pop();

    if (id == id2) {
      found = true;
      break;
    }

    // max search time
    // if ((ros::Time::now() - start_time).toSec() > max_search_time_) {
    //   ROS_ERROR("[ConnectivityGraph] Search time exceeds the maximum search time");
    //   return -1.0;
    // }

    for (auto it = nodes_[id]->neighbors_.begin(); it != nodes_[id]->neighbors_.end(); ++it) {
      if (visited_flags[it->id2_] || it->cost_ > 499.0) {
        continue;
      }

      // std::cout << "Current node: " << id << ", neighbor: " << it->id2_ << ", cost: " <<
      // it->cost_
      //           << std::endl;

      queue.push(it->id2_);
      visited_flags[it->id2_] = true;
      parent_map.insert(std::make_pair(it->id2_, id));
    }
  }

  if (!found) {
    // ROS_ERROR("[ConnectivityGraph] No path found for id1 %d and id2 %d", id1, id2);
    return 1000.0;
  }

  int id = id2;
  while (id != -1) {
    path.push_back(id);
    id = parent_map[id];
  }

  std::reverse(path.begin(), path.end());

  double cost = 0.0;
  for (int i = 0; i < path.size() - 1; ++i) {
    for (auto it = nodes_[path[i]]->neighbors_.begin(); it != nodes_[path[i]]->neighbors_.end();
         ++it) {
      if (it->id2_ == path[i + 1]) {
        cost += it->cost_;
        break;
      }
    }
  }

  if (cost < 1e-6) {
    ROS_ERROR("[ConnectivityGraph] Path cost is zero");
    // print start end
    std::cout << "start: " << id1 << ", end: " << id2 << std::endl;
    // print path
    for (int i = 0; i < path.size(); ++i) {
      std::cout << path[i] << " ";
    }
  }

  return cost;
}

// TODO: Implement this function if BFS is not fast enough
double ConnectivityGraph::searchConnectivityGraphAStar(const int &id1, const int &id2,
                                                       std::vector<int> &path) {
  CHECK(false) << "Not implemented";
}

void ConnectivityGraph::getFullConnectivityGraph(std::vector<std::pair<Position, Position>> &edges,
                                                 std::vector<ConnectivityEdge::TYPE> &edges_types,
                                                 std::vector<double> &edges_costs) {
  edges.clear();
  edges_types.clear();

  std::unordered_map<int, bool> visited_flags;
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    visited_flags.insert(std::make_pair(it->first, false));
  }
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    for (auto it2 = it->second->neighbors_.begin(); it2 != it->second->neighbors_.end(); ++it2) {
      if (visited_flags[it2->id2_]) {
        continue;
      }

      if (it2->cost_ > 499.0) {
        continue;
      }

      edges.push_back(std::make_pair(it->second->pos_, nodes_[it2->id2_]->pos_));
      edges_types.push_back(it2->type_);
      edges_costs.push_back(it2->cost_);
    }

    visited_flags[it->first] = true;
  }

  CHECK_EQ(edges.size(), edges_types.size()) << "edges and edges_types have different sizes";
  CHECK_EQ(edges.size(), edges_costs.size()) << "edges and edges_costs have different sizes";
}

void ConnectivityGraph::getFullConnectivityGraphPath(
    std::vector<std::vector<Position>> &paths, std::vector<ConnectivityEdge::TYPE> &paths_types) {
  paths.clear();
  paths_types.clear();

  std::unordered_map<int, bool> visited_flags;
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    visited_flags.insert(std::make_pair(it->first, false));
  }

  // std::cout << "nodes_.size() = " << nodes_.size() << std::endl;
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    // std::cout << "node id = " << it->second->id_
    //           << ", neighbors_.size() = " << it->second->neighbors_.size() << std::endl;
    for (auto it2 = it->second->neighbors_.begin(); it2 != it->second->neighbors_.end(); ++it2) {
      if (visited_flags[it2->id2_]) {
        continue;
      }

      if (it2->cost_ > 499.0) {
        continue;
      }

      if (it2->path_.empty()) {
        Position pos1, pos2;
        pos1 = it->second->pos_;
        pos2 = nodes_[it2->id2_]->pos_;
        std::vector<Position> path;
        path.push_back(pos1);
        path.push_back(pos2);
        paths.push_back(path);
      } else {
        paths.push_back(it2->path_);
      }
      paths_types.push_back(it2->type_);

      // std::cout << "add path from " << it->second->id_ << " to " << nodes_[it2->id2_]->id_
      //           << std::endl;
    }

    visited_flags[it->first] = true;
  }
  // std::cout << "paths.size() = " << paths.size() << std::endl;
}

void ConnectivityGraph::findDisconnectedNodes(std::set<int> &disconnected_nodes) {
  // Using connected components algorithm to find disconnected nodes
  // https://en.wikipedia.org/wiki/Connected-component_labeling
  ros::Time t1 = ros::Time::now();

  disconnected_nodes.clear();
  std::unordered_map<int, int> labels;
  // init labels for all nodes
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    labels.insert(std::make_pair(it->first, -1));
  }

  int label = 0;
  vector<int> ids;
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    if (labels[it->first] != -1) {
      continue;
    }
    std::stack<int> stack;
    stack.push(it->first);
    labels[it->first] = label;

    ids.clear();
    ids.push_back(it->first);

    bool isAllUnkown = true;
    while (!stack.empty()) {
      int id = stack.top();
      stack.pop();
      for (auto it2 = nodes_[id]->neighbors_.begin(); it2 != nodes_[id]->neighbors_.end(); ++it2) {
        if (labels[it2->id2_] != -1) {
          continue;
        }

        if (it2->cost_ > 499.0) {
          continue;
        }

        stack.push(it2->id2_);
        labels[it2->id2_] = label;
        ids.push_back(it2->id2_);

        if (nodes_[it2->id2_]->type_ != ConnectivityNode::TYPE::UNKNOWN)
          isAllUnkown = false;
      }
    }

    label++;

    if (isAllUnkown) {
      for (int i = 0; i < ids.size(); ++i) {
        disconnected_nodes.insert(ids[i]);
      }
    }

    // // Print ids for label
    // std::cout << "label " << label << ": ";
    // for (int i = 0; i < ids.size(); ++i) {
    //   std::cout << ids[i] << " ";
    // }
    // if (isAllUnkown)
    //   std::cout << " (all unknown)";
    // else
    //   std::cout << " (not all unknown)";
    // std::cout << std::endl;
  }

  // time cost
  ros::Time t2 = ros::Time::now();
  // std::cout << "time cost for finding disconnected nodes: " << (t2 - t1).toSec() << std::endl;
}

bool ConnectivityGraph::saveConnectivityGraph(const std::string &file_path) {
  std::ofstream file(file_path);
  if (!file.is_open()) {
    ROS_ERROR("[ConnectivityGraph] Cannot open file %s", file_path.c_str());
    return false;
  }

  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    file << it->second->id_ << " " << it->second->pos_.transpose() << " " << (int)it->second->type_
         << std::endl;
    for (auto it2 = it->second->neighbors_.begin(); it2 != it->second->neighbors_.end(); ++it2) {
      file << it2->id2_ << " " << it2->cost_ << " " << (int)it2->type_ << std::endl;
    }
    file << std::endl;
  }

  file.close();
  return true;
}

bool ConnectivityGraph::loadConnectivityGraph(const std::string &file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    ROS_ERROR("[ConnectivityGraph] Cannot open file %s", file_path.c_str());
    return false;
  }

  clearNodes();

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    int id;
    Position pos;
    int type;
    iss >> id >> pos.x() >> pos.y() >> pos.z() >> type;
    ConnectivityNode::Ptr node(
        new ConnectivityNode(id, pos, static_cast<ConnectivityNode::TYPE>(type)));
    while (std::getline(file, line)) {
      if (line.empty()) {
        break;
      }
      std::istringstream iss2(line);
      int id2;
      double cost;
      int type;
      iss2 >> id2 >> cost >> type;
      // temporarily times cost by 2.0 to eliminate the effect of the max velocity
      node->addNeighbor(id2, cost * 2.0, static_cast<ConnectivityEdge::TYPE>(type));
    }
    addNode(node);
  }

  file.close();
  return true;
}

} // namespace fast_planner