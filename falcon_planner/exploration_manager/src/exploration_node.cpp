#include <glog/logging.h>
#include <ros/ros.h>
#include <ros/package.h>

#include "backward/backward.hpp"
#include "exploration_manager/exploration_fsm.h"
#include "system_info.h"

namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  string glog_path;
  glog_path = ros::package::getPath("exploration_manager") + "/log";
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = glog_path;
  FLAGS_minloglevel = google::INFO;
  LOG(INFO) << "Exploration task start at " << ros::Time::now();

  string system_info;
  printSystemInfo(system_info);
  std::cout << system_info;
  LOG(INFO) << "\n" << system_info;

  // Check if use_sim_time is false
  bool use_sim_time;
  nh.param("/use_sim_time", use_sim_time, false);
  CHECK(!use_sim_time) << "Please set use_sim_time to false";

  ExplorationFSM fsm;
  fsm.init(nh);
  ros::Duration(1.0).sleep(); // Wait for initialization

  ros::spin();

  google::ShutdownGoogleLogging();

  return 0;
}
