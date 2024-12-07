
#include "exploration_manager/exploration_fsm.h"
#include "exploration_manager/exploration_manager.h"
#include "exploration_manager/exploration_data.h"
#include "fast_planner/planner_manager.h"
#include "tic_toc.h"
#include "traj_utils/planning_visualization.h"

using Eigen::Vector4d;

namespace fast_planner {
void ExplorationFSM::init(ros::NodeHandle &nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("/exploration_manager/fsm/replan_thresh1", fp_->replan_thresh1_, -1.0);
  nh.param("/exploration_manager/fsm/replan_thresh2", fp_->replan_thresh2_, -1.0);
  nh.param("/exploration_manager/fsm/replan_thresh3", fp_->replan_thresh3_, -1.0);
  nh.param("/exploration_manager/fsm/replan_duration_fast", fp_->replan_duration_fast_, -1.0);
  nh.param("/exploration_manager/fsm/replan_duration_default", fp_->replan_duration_default_, -1.0);
  nh.param("/exploration_manager/fsm/replan_duration_slow", fp_->replan_duration_slow_, -1.0);
  fp_->replan_duration_ = fp_->replan_duration_default_;

  /* Initialize main modules */
  expl_manager_.reset(new ExplorationManager);
  expl_manager_->initialize(nh);
  expl_manager_->ep_->replan_duration_ = fp_->replan_duration_;
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  expl_manager_->visualization_ = visualization_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH", "RTB"};
  fd_->static_state_ = true;
  fd_->triggered_ = false;

  frontier_ready_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ =
      nh.createTimer(ros::Duration(0.01), &ExplorationFSM::FSMCallback, this); // 100Hz
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &ExplorationFSM::safetyCallback, this);
  frontier_timer_ =
      nh.createTimer(ros::Duration(0.5), &ExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/move_base_simple/goal", 1, &ExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &ExplorationFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Int32>("/planning/replan", 10);
  bspline_pub_ = nh.advertise<trajectory::Bspline>("/planning/bspline", 10);
  uncertainty_pub_ = nh.advertise<std_msgs::Float32>("/planning/uncertainty", 10);
}

void ExplorationFSM::FSMCallback(const ros::TimerEvent &e) {
  LOG(INFO) << "--------------------START OF CALLBACK--------------------";

  // Current time in sec
  // ROS_INFO_THROTTLE(1.0, "[FSM] Current time: %.2lf", ros::Time::now().toSec());

  // if (state_ == INIT || state_ == WAIT_TRIGGER || state_ == FINISH)
  //   ROS_INFO("[FSM] Current state: %s", fd_->state_str_[int(state_)].c_str());
  // else
  //   ROS_INFO_THROTTLE(1.0, "[FSM] Current state: %s", fd_->state_str_[int(state_)].c_str());

  // Log current state and calculate avg callback rate
  static int cnt_callback = 0;
  static ros::Time start_time = ros::Time::now();
  static ros::Time last_time = ros::Time::now();
  double dt = (ros::Time::now() - start_time).toSec();
  LOG(INFO) << "[FSM] Current state: " << fd_->state_str_[int(state_)]
            << ", callback rate: " << cnt_callback / dt << "Hz";
  if (dt > 1.0) {
    start_time = ros::Time::now();
    cnt_callback = 0;
  }
  cnt_callback++;

  switch (state_) {
  case INIT: {
    // Wait for odometry ready
    if (!fd_->have_odom_) {
      ROS_INFO_THROTTLE(1.0, "[FSM] No odom");
      return;
    }
    if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0) {
      ROS_INFO_THROTTLE(1.0, "[FSM] Initializing");
      return;
    }
    // Go to wait trigger when odom is ok
    ROS_WARN("[FSM] Receive odom from topic %s", odom_sub_.getTopic().c_str());
    LOG(WARNING) << "[FSM] Receive odom from topic " << odom_sub_.getTopic().c_str();
    transitState(WAIT_TRIGGER, "FSM");
    break;
  }

  case WAIT_TRIGGER: {
    // Update frontiers(in callback) and hgrid when waiting for trigger
    if (frontier_ready_)
      expl_manager_->initializeHierarchicalGrid(fd_->odom_pos_, fd_->odom_vel_);

    visualize();

    break;
  }

  case FINISH: {
    ROS_INFO_ONCE("[FSM] Exploration finished");

    std_msgs::Int32 replan_msg;
    replan_msg.data = 2;
    replan_pub_.publish(replan_msg);

    static bool clear_vis = false;
    if (!clear_vis) {
      auto ed_ = expl_manager_->ed_;
      ed_->n_points_.clear();
      ed_->refined_ids_.clear();
      ed_->unrefined_points_.clear();
      ed_->refined_points_.clear();
      ed_->refined_views_.clear();
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
      ed_->refined_tour_.clear();
      ed_->grid_tour_.clear();
      ed_->grid_tour2_.clear();
      ed_->grid_tour3_.clear();
      visualize();
      clear_vis = true;

      double map_coverage = planner_manager_->map_server_->getMapCoverage();
      expl_manager_->ed_->finish_time_ = ros::Time::now();
      ROS_INFO("[FSM] Exploration finished. Start time: %.2lf, End time: %.2lf, Duration: %.2lf, "
               "Coverage: %.2lf",
               expl_manager_->ed_->start_time_.toSec(), expl_manager_->ed_->finish_time_.toSec(),
               (expl_manager_->ed_->finish_time_ - expl_manager_->ed_->start_time_).toSec(),
               map_coverage);

      if (false) {
        // Save times to /home/eason/workspace/exploration_ws/times.csv
        string output_file = "/home/eason/workspace/exploration_ws/times.csv";
        ofstream fout(output_file, ios::app);
        if (!fout.is_open()) {
          ROS_ERROR("[FSM] Cannot open file %s", output_file.c_str());
          return;
        }

        // std::vector<double> frontier_times_;
        // std::vector<double> space_decomp_times_;
        // std::vector<double> connectivity_graph_times_;
        // std::vector<std::pair<double, double>> cp_times_;
        // std::vector<std::pair<double, double>> sop_times_;

        for (int i = 0; i < expl_manager_->ee_->frontier_times_.size(); ++i) {
          fout << expl_manager_->ee_->frontier_times_[i] << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->total_times_.size(); ++i) {
          fout << expl_manager_->ee_->total_times_[i] << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->space_decomp_times_.size(); ++i) {
          fout << expl_manager_->ee_->space_decomp_times_[i] << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->connectivity_graph_times_.size(); ++i) {
          fout << expl_manager_->ee_->connectivity_graph_times_[i] << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->cp_times_.size(); ++i) {
          fout << expl_manager_->ee_->cp_times_[i].first << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->cp_times_.size(); ++i) {
          fout << expl_manager_->ee_->cp_times_[i].second << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->sop_times_.size(); ++i) {
          fout << expl_manager_->ee_->sop_times_[i].first << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->sop_times_.size(); ++i) {
          fout << expl_manager_->ee_->sop_times_[i].second << ", ";
        }
        fout << endl;
        fout.close();

        // calculate mean and std and max and min for each times vector
        auto calc_mean_std = [](const std::vector<double> &vec, double &mean, double &std,
                                double &max, double &min) {
          mean = 0.0;
          std = 0.0;
          max = -1e10;
          min = 1e10;
          for (auto v : vec) {
            mean += v;
            max = std::max(max, v);
            min = std::min(min, v);
          }
          mean /= vec.size();
          for (auto v : vec) {
            std += (v - mean) * (v - mean);
          }
          std = std::sqrt(std / vec.size());
        };
        auto calc_mean_std2 = [](const std::vector<std::pair<double, double>> &vec,
                                pair<double, double> &mean, pair<double, double> &std,
                                pair<double, double> &max, pair<double, double> &min) {
          mean.first = 0.0;
          mean.second = 0.0;
          std.first = 0.0;
          std.second = 0.0;
          max.first = -1e10;
          max.second = -1e10;
          min.first = 1e10;
          min.second = 1e10;
          for (auto v : vec) {
            mean.first += v.first;
            mean.second += v.second;
            max.first = std::max(max.first, v.first);
            max.second = std::max(max.second, v.second);
            min.first = std::min(min.first, v.first);
            min.second = std::min(min.second, v.second);
          }
          mean.first /= vec.size();
          mean.second /= vec.size();
          for (auto v : vec) {
            std.first += (v.first - mean.first) * (v.first - mean.first);
            std.second += (v.second - mean.second) * (v.second - mean.second);
          }
          std.first = std::sqrt(std.first / vec.size());
          std.second = std::sqrt(std.second / vec.size());
        };

        double mean, std, max, min;
        calc_mean_std(expl_manager_->ee_->frontier_times_, mean, std, max, min);
        ROS_INFO("[FSM] Frontier times: mean %.6lf, std %.6lf, max %.6lf, min %.6lf", mean, std, max,
                min);
        calc_mean_std(expl_manager_->ee_->total_times_, mean, std, max, min);
        ROS_INFO("[FSM] Total times: mean %.6lf, std %.6lf, max %.6lf, min %.6lf", mean, std, max,
                min);
        calc_mean_std(expl_manager_->ee_->space_decomp_times_, mean, std, max, min);
        ROS_INFO("[FSM] Space decomp times: mean %.6lf, std %.6lf, max %.6lf, min %.6lf", mean, std,
                max, min);
        calc_mean_std(expl_manager_->ee_->connectivity_graph_times_, mean, std, max, min);
        ROS_INFO("[FSM] Connectivity graph times: mean %.6lf, std %.6lf, max %.6lf, min %.6lf", mean,
                std, max, min);

        pair<double, double> mean2, std2, max2, min2;
        calc_mean_std2(expl_manager_->ee_->cp_times_, mean2, std2, max2, min2);
        ROS_INFO(
            "[FSM] CP times: mean %.6lf, %.6lf, std %.6lf, %.6lf, max %.6lf, %.6lf, min %.6lf, %.6lf",
            mean2.first, mean2.second, std2.first, std2.second, max2.first, max2.second, min2.first,
            min2.second);
        calc_mean_std2(expl_manager_->ee_->sop_times_, mean2, std2, max2, min2);
        ROS_INFO("[FSM] SOP times: mean %.6lf, %.6lf, std %.6lf, %.6lf, max %.6lf, %.6lf, min %.6lf, "
                "%.6lf",
                mean2.first, mean2.second, std2.first, std2.second, max2.first, max2.second,
                min2.first, min2.second);
      }
    }

    // if (ros::Time::now() - expl_manager_->ed_->finish_time_ > ros::Duration(5.0))
    //   transitState(RTB, "FSM");

    break;
  }

  case RTB: {
    // TODO
    ROS_INFO_ONCE("[FSM] Return to base");
    break;
  }

  case PLAN_TRAJ: {
    int cur_cell_id, cur_center_id;
    expl_manager_->hierarchical_grid_->getLayerPositionCellCenterId(0, fd_->odom_pos_, cur_cell_id,
                                                                    cur_center_id);
    ROS_INFO("[FSM] Current cell id: %d, center id: %d", cur_cell_id, cur_center_id);

    // if (cur_cell_id == -1 || cur_center_id == -1) {
    //   CHECK(false) << "Invalid current cell id or center id";
    // }

    double map_coverage = planner_manager_->map_server_->getMapCoverage();
    ROS_BLUE_STREAM("[FSM] Exploration planning. Start time: "
                    << expl_manager_->ed_->start_time_ << ", Current time: " << ros::Time::now()
                    << ", Duration: " << (ros::Time::now() - expl_manager_->ed_->start_time_)
                    << ", Coverage: " << map_coverage);

    if (fd_->static_state_) {
      // Plan from static state (hover)
      fd_->start_pos_ = fd_->odom_pos_;
      fd_->start_vel_ = fd_->odom_vel_;
      fd_->start_acc_.setZero();
      fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
      trajectory_start_time_ = ros::Time::now() + ros::Duration(fp_->replan_duration_);
    } else {
      // Replan from non-static state, starting from 'replan_time' seconds later
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_r = (time_now - info->start_time_).toSec() + fp_->replan_duration_;
      if (t_r > info->duration_) {
        t_r = info->duration_;
      }
      trajectory_start_time_ = time_now + ros::Duration(fp_->replan_duration_);
      fd_->start_pos_ = info->position_traj_.evaluateDeBoorT(t_r);
      fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
      fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
      fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
      fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
      fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
    }

    if (false) {
      // Print current states
      ROS_INFO("[FSM] Current pos: %.2lf, %.2lf, %.2lf", fd_->odom_pos_[0], fd_->odom_pos_[1],
               fd_->odom_pos_[2]);
      ROS_INFO("[FSM] Current vel: %.2lf, %.2lf, %.2lf", fd_->odom_vel_[0], fd_->odom_vel_[1],
               fd_->odom_vel_[2]);
      ROS_INFO("[FSM] Current yaw: %.2lf", fd_->odom_yaw_);

      LOG(INFO) << "[FSM] Current pos: " << fd_->odom_pos_.transpose()
                << ", vel: " << fd_->odom_vel_.transpose() << ", yaw: " << fd_->odom_yaw_;

      // Print start states
      ROS_INFO("[FSM] Start pos: %.2lf, %.2lf, %.2lf", fd_->start_pos_[0], fd_->start_pos_[1],
               fd_->start_pos_[2]);
      ROS_INFO("[FSM] Start vel: %.2lf, %.2lf, %.2lf", fd_->start_vel_[0], fd_->start_vel_[1],
               fd_->start_vel_[2]);
      ROS_INFO("[FSM] Start acc: %.2lf, %.2lf, %.2lf", fd_->start_acc_[0], fd_->start_acc_[1],
               fd_->start_acc_[2]);
      ROS_INFO("[FSM] Start yaw: %.2lf, %.2lf, %.2lf", fd_->start_yaw_[0], fd_->start_yaw_[1],
               fd_->start_yaw_[2]);

      LOG(INFO) << "[FSM] Start pos: " << fd_->start_pos_.transpose()
                << ", vel: " << fd_->start_vel_.transpose()
                << ", acc: " << fd_->start_acc_.transpose()
                << ", yaw: " << fd_->start_yaw_.transpose();

      for (int i = 0; i < 3; ++i) {
        if (abs(fd_->start_vel_[i]) > planner_manager_->pp_.max_vel_) {
          ROS_WARN("[FSM] Start vel too high: %.2lf", fd_->start_vel_[i]);
        }

        if (abs(fd_->start_acc_[i]) > planner_manager_->pp_.max_acc_) {
          ROS_WARN("[FSM] Start acc too high: %.2lf", fd_->start_acc_[i]);
        }
      }
    }

    // Inform traj_server the replanning
    std_msgs::Int32 replan_msg;
    replan_msg.data = 0;
    replan_pub_.publish(replan_msg);

    // Exploration plannner main function
    int res = callExplorationPlanner();
    if (res == SUCCEED) {
      transitState(PUB_TRAJ, "FSM");
    } else if (res == FAIL) { // Keep trying to replan
      fd_->static_state_ = true;
      ROS_WARN("[FSM] Plan fail");
    } else if (res == NO_GRID) {
      fd_->static_state_ = true;
      ROS_WARN("[FSM] Finish exploration: No grid");

      CHECK_EQ(expl_manager_->ed_->frontiers_.size(), 0)
          << "Frontier size: " << expl_manager_->ed_->frontiers_.size() << ", but no grid";

      transitState(FINISH, "FSM");
    }

    thread vis_thread(&ExplorationFSM::visualize, this);
    vis_thread.detach();

    break;
  }

  case PUB_TRAJ: {
    bool safe = planner_manager_->checkTrajCollision();
    // if (!safe) {
    //   ROS_ERROR("[FSM] Replan: collision detected on the trajectory before publishing");
    //   fd_->static_state_ = true;
    //   transitState(PLAN_TRAJ, "FSM");
    //   break;
    // }

    if (!safe) {
      ROS_ERROR("[FSM] Collision detected on the trajectory before publishing");

      planner_manager_->local_data_.position_traj_ =
          planner_manager_->local_data_.init_traj_bspline_;
      planner_manager_->local_data_.velocity_traj_ =
          planner_manager_->local_data_.position_traj_.getDerivative();
      planner_manager_->local_data_.acceleration_traj_ =
          planner_manager_->local_data_.velocity_traj_.getDerivative();
      planner_manager_->local_data_.duration_ =
          planner_manager_->local_data_.position_traj_.getTimeSum();
      planner_manager_->local_data_.start_pos_ =
          planner_manager_->local_data_.position_traj_.evaluateDeBoorT(0.0);
      planner_manager_->local_data_.end_pos_ =
          planner_manager_->local_data_.position_traj_.evaluateDeBoorT(
              planner_manager_->local_data_.duration_);
      planner_manager_->local_data_.start_yaw_ =
          planner_manager_->local_data_.yaw_traj_.evaluateDeBoorT(0.0)[0];
      planner_manager_->local_data_.end_yaw_ =
          planner_manager_->local_data_.yaw_traj_.evaluateDeBoorT(
              planner_manager_->local_data_.duration_)[0];

      bool safe_init_path = planner_manager_->checkTrajCollision();

      if (!safe_init_path) {
        ROS_ERROR("[FSM] Replan: collision also detected on the initial trajectory");

        fd_->static_state_ = true;
        transitState(PLAN_TRAJ, "FSM");
        break;
      }

      ROS_WARN("[FSM] Replacing the trajectory with the safe initial trajectory");
    }

    // Check traj avg vel
    double pos_traj_length = planner_manager_->local_data_.position_traj_.getLength();
    double pos_traj_duration = planner_manager_->local_data_.position_traj_.getTimeSum();
    double avg_pos_vel = pos_traj_length / pos_traj_duration;
    double yaw_traj_length = planner_manager_->local_data_.yaw_traj_.getLength();
    double yaw_traj_duration = planner_manager_->local_data_.yaw_traj_.getTimeSum();
    double avg_yaw_vel = yaw_traj_length / yaw_traj_duration;
    // ROS_WARN_COND(avg_vel < 0.1, "[FSM] Average velocity too low: %.2lf", avg_vel);

    // static bool traj_lengthened = false;
    // if (avg_pos_vel < 0.5 && avg_yaw_vel < 0.5 && !traj_lengthened) {
    if (avg_pos_vel < 0.5 && avg_yaw_vel < 0.5) {
      ROS_WARN("[FSM] Slow trajectory detected, duration: %.2lf, length: %.2lf", pos_traj_duration,
               pos_traj_length);
      // traj_lengthened = true;
      double yaw_ratio = 1.57 / avg_yaw_vel;
      double pos_ratio = 2.0 / avg_pos_vel;
      double ratio = 1.0 / std::min(yaw_ratio, pos_ratio);
      planner_manager_->local_data_.position_traj_.lengthenTime(ratio);
      planner_manager_->local_data_.yaw_traj_.lengthenTime(ratio);
      ROS_WARN("[FSM] Avg position velocity: %.2lf, avg yaw velocity: %.2lf, lengthen ratio: %.2lf",
               avg_pos_vel, avg_yaw_vel, ratio);

      double init_traj_duration = planner_manager_->local_data_.init_traj_bspline_.getTimeSum();
      ROS_WARN("[FSM] Initial traj duration: %.2lf", init_traj_duration);
    }

    trajectory::Bspline bspline;
    auto info = &planner_manager_->local_data_;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;

    double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
    // ROS_BLUE_STREAM("[FSM] Traj dt: " << dt);
    if (dt > 0.0) {
      bspline_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      transitState(EXEC_TRAJ, "FSM");

      // traj_lengthened = false;

      // thread vis_thread(&ExplorationFSM::visualize, this);
      // vis_thread.detach();
    }
    break;
  }

  case EXEC_TRAJ: {
    auto tn = ros::Time::now();
    // Check whether replan is needed
    LocalTrajData *info = &planner_manager_->local_data_;
    double t_cur = (tn - info->start_time_).toSec();

    // if (!fd_->go_back_) {
    bool need_replan = false;
    int replan_type = 0;
    if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
      // Replan if frontier cluster is covered with some percentage
      ROS_WARN("[FSM] Replan: cluster covered");
      need_replan = true;
      replan_type = 1;
    } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
      // Replan if traj is almost fully executed
      ROS_WARN("[FSM] Replan: traj fully executed");
      need_replan = true;
      replan_type = 2;
    } else if (t_cur > fp_->replan_thresh3_) {
      // Replan after some time
      ROS_WARN("[FSM] Replan: periodic call");
      need_replan = true;
      replan_type = 3;
    }

    if (need_replan) {
      {
        // Log replan type
        string replan_type_str;
        switch (replan_type) {
        case 0: {
          replan_type_str = "no replan";
          break;
        }
        case 1: {
          replan_type_str = "cluster covered";
          break;
        }
        case 2: {
          replan_type_str = "traj fully executed";
          break;
        }
        case 3: {
          replan_type_str = "periodic call";
          break;
        }
        }
        LOG(INFO) << "[FSM] Replan type: " << replan_type_str;

        // Visualize replan type in Rviz
        visualization_->drawText(Eigen::Vector3d(12, 0, 0), replan_type_str, 1,
                                 PlanningVisualization::Color::Black(), "replan_type", 0,
                                 PlanningVisualization::PUBLISHER::DEBUG);
      }

      if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
        // Update frontier and plan new motion
        thread vis_thread(&ExplorationFSM::visualize, this);
        vis_thread.detach();

        transitState(PLAN_TRAJ, "FSM");

        // Use following code can debug the planner step by step
        // transitState(WAIT_TRIGGER, "FSM");
        // fd_->static_state_ = true;

      } else {
        // No frontier detected, finish exploration
        transitState(FINISH, "FSM");
        ROS_WARN("[FSM] Finish exploration: No frontier detected");
        // clearVisMarker();
        // visualize();
      }
    }

    break;
  }
  }

  LOG(INFO) << "[FSM] Callback time: " << (ros::Time::now() - last_time).toSec() << "s";
  LOG(INFO) << "--------------------END OF CALLBACK--------------------";
  LOG(INFO) << "";

  last_time = ros::Time::now();
}

int ExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_duration_);

  ros::Time plan_start_time = ros::Time::now();
  int res = expl_manager_->planExploreMotionHGrid(fd_->start_pos_, fd_->start_vel_, fd_->start_acc_,
                                                  fd_->start_yaw_);
  ros::Time plan_end_time = ros::Time::now();
  double plan_total_time = (ros::Time::now() - plan_start_time).toSec();
  ROS_BLUE_STREAM("[FSM] Exploration planner total time: "
                  << std::setprecision(4) << plan_total_time * 1000.0
                  << "ms, replan duration: " << fp_->replan_duration_ * 1000.0 << "ms");
  ROS_ERROR_COND(plan_total_time > fp_->replan_duration_, "[FSM] Total time too long!");

  // Dynamic replan duration
  if (plan_total_time > fp_->replan_duration_slow_) {
    fp_->replan_duration_ = fp_->replan_duration_slow_;
  } else if (plan_total_time < fp_->replan_duration_fast_) {
    fp_->replan_duration_ = fp_->replan_duration_fast_;
  } else {
    fp_->replan_duration_ = fp_->replan_duration_default_;
  }
  expl_manager_->ep_->replan_duration_ = fp_->replan_duration_;

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    // ROS_ERROR_COND((ros::Time::now() - time_r).toSec() < -0.1,
    //                "[FSM] replan_time set too large, please decrease it. diff: %lf",
    //                (ros::Time::now() - time_r).toSec());
    // ROS_ERROR_COND((ros::Time::now() - time_r).toSec() > 0.1,
    //                "[FSM] replan_time set too small, please increase it. diff: %lf",
    //                (ros::Time::now() - time_r).toSec());
    // info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
    info->start_time_ = trajectory_start_time_;
  }

  return res;
}

void ExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->map_server_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3),
  // "updated_box", 0, 4);

  // New frontier visualization, using pointcloud instead of marker
  double map_resolution = planner_manager_->map_server_->getResolution();
  if (map_resolution > 0.1) {
    visualization_->drawFrontierPointcloudHighResolution(ed_ptr->frontiers_, map_resolution);
    visualization_->drawDormantFrontierPointcloudHighResolution(ed_ptr->dormant_frontiers_,
                                                                map_resolution);
    visualization_->drawTinyFrontierPointcloudHighResolution(ed_ptr->tiny_frontiers_,
                                                             map_resolution);
  } else {
    visualization_->drawFrontierPointcloud(ed_ptr->frontiers_);
    visualization_->drawDormantFrontierPointcloud(ed_ptr->dormant_frontiers_);
    visualization_->drawTinyFrontierPointcloud(ed_ptr->tiny_frontiers_);
  }

  vector<int> visib_nums;
  expl_manager_->frontier_finder_->getDormantFrontiersVisibleNums(visib_nums);

  static std::vector<int> last_frontier_ids;
  // cout << "Clear previous frontier ids" << endl;
  for (auto i : last_frontier_ids) {
    visualization_->drawText(Eigen::Vector3d::Zero(), "", 0.5, PlanningVisualization::Color::Red(),
                             "frontier_id", i, PlanningVisualization::PUBLISHER::FRONTIER);
  }
  last_frontier_ids.clear();
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawText(ed_ptr->averages_[i], to_string(i), 0.5,
                             PlanningVisualization::Color::Red(), "frontier_id", i,
                             PlanningVisualization::PUBLISHER::FRONTIER);
    last_frontier_ids.push_back(i);
  }

  static std::vector<int> last_dormant_frontier_ids;
  // cout << "Clear previous dormant frontier ids" << endl;
  for (auto i : last_dormant_frontier_ids) {
    visualization_->drawText(Eigen::Vector3d::Zero(), "", 0.5, PlanningVisualization::Color::Red(),
                             "frontier_visble_num", i, PlanningVisualization::PUBLISHER::FRONTIER);
  }
  last_dormant_frontier_ids.clear();
  for (int i = 0; i < ed_ptr->dormant_frontiers_.size(); ++i) {
    visualization_->drawText(ed_ptr->dormant_frontiers_[i][0], to_string(visib_nums[i]), 1.0,
                             PlanningVisualization::Color::Red(), "frontier_visble_num", i,
                             PlanningVisualization::PUBLISHER::FRONTIER);
    last_dormant_frontier_ids.push_back(i);
  }

  visualization_->drawLines(ed_ptr->grid_tour2_, 0.1, PlanningVisualization::Color::Blue2(),
                            "hgrid_tour_mc", 0, PlanningVisualization::PUBLISHER::HGRID);

  // Draw global top viewpoints info
  visualization_->drawSpheres(ed_ptr->points_, 0.2, PlanningVisualization::Color::DeepGreen(),
                              "points", 0, PlanningVisualization::PUBLISHER::VIEWPOINT);

  visualization_->drawLines(ed_ptr->views1_, ed_ptr->views2_, 0.02,
                            PlanningVisualization::Color::Black(), "view_fov", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw a line from frontier top view point with its view direction
  visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05,
                            PlanningVisualization::Color::Yellow(), "frontier_view", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);
  // Draw a line from frontier top view point to frontier average center
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03,
  //                           PlanningVisualization::Color::DeepGreen(), "point_average", 0,
  //                           PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw local refined viewpoints info
  visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, PlanningVisualization::Color::Blue(),
                              "refined_pts", 0, 6);

  visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
                            PlanningVisualization::Color::LightBlue(), "refined_view", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);
  visualization_->drawLines(ed_ptr->refined_tour_, 0.07, PlanningVisualization::Color::DeepBlue(),
                            "refined_tour", 0, 6);

  visualization_->drawLines(ed_ptr->path_next_goal_, 0.07, PlanningVisualization::Color::DeepBlue(),
                            "path_next_goal", 0, 6);

  // Draw refined view FOV
  visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.02,
                            PlanningVisualization::Color::Black(), "refined_view_fov", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw a line to show the pair of the original view point and refined view point
  visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.02,
                            PlanningVisualization::Color::Yellow(), "refine_pair", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);

  // visualization_->drawLines(ed_ptr->n_views1_, ed_ptr->n_views2_, 0.02,
  //                           PlanningVisualization::Color::Black(), "n_views_fov", 0,
  //                           PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw sampled viewpoints
  for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
    visualization_->drawSpheres(
        ed_ptr->n_points_[i], 0.2,
        visualization_->getColor(double(ed_ptr->refined_ids_[i]) / ed_ptr->frontiers_.size()),
        "n_points", i, PlanningVisualization::PUBLISHER::VIEWPOINT);
  // Remove markers if this time has less #frontiers than previous time
  for (int i = ed_ptr->n_points_.size(); i < 20; ++i)
    visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i,
                                PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw next goal position
  Eigen::Quaterniond next_q;
  next_q = Eigen::AngleAxisd(ed_ptr->next_yaw_, Eigen::Vector3d::UnitZ());
  visualization_->drawPose(ed_ptr->next_goal_, next_q, "next_goal", 0);
  // visualization_->drawSpheres({ed_ptr->next_goal_}, 1.0,
  //                             PlanningVisualization::Color::TealTransparent(), "next_goal", 0,
  //                             PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw trajectory
  // visualization_->drawPolynomialTraj(info->init_traj_poly_, 0.1,
  // PlanningVisualization::Color::Red(), 0); visualization_->drawPolynomialTraj(info->init_traj2_,
  // 0.1, PlanningVisualization::Color::Red(), 1);
  if (info->position_traj_.getKnot().size() > 0) {
    visualization_->drawBspline(planner_manager_->local_data_.init_traj_bspline_, 0.05,
                                PlanningVisualization::Color::DeepGreen(), true, 0.1,
                                PlanningVisualization::Color::DeepGreen(), 0);
    visualization_->drawBspline(planner_manager_->local_data_.position_traj_, 0.05,
                                PlanningVisualization::Color::DeepBlue(), true, 0.1,
                                PlanningVisualization::Color::DeepBlue(), 1);
    // visualization_->drawBsplineWithVelocity(info->position_traj_, info->velocity_traj_);
  }

  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1,
  // 6);

  // expl_manager_->hierarchical_grid_->publishGridsLayer();
  // expl_manager_->hierarchical_grid_->publishGridsConnectivityLayer();

  // Draw frontier first pc
  // vector<Vector2d> first_pcs;
  // vector<Position> first_pcs_3d;
  // expl_manager_->frontier_finder_->getFrontierFirstPCs(first_pcs);

  // first_pcs_3d.resize(first_pcs.size());
  // for (int i = 0; i < first_pcs.size(); ++i) {
  //   first_pcs_3d[i] = ed_ptr->averages_[i] + Vector3d(first_pcs[i](0), first_pcs[i](1), 0);
  // }

  // visualization_->drawLines(ed_ptr->averages_, first_pcs_3d, 0.02,
  //                           PlanningVisualization::Color::White(), "frontier_first_pc", 0,
  //                           PlanningVisualization::PUBLISHER::FRONTIER);
}

void ExplorationFSM::clearVisMarker() {
  visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);
  visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void ExplorationFSM::frontierCallback(const ros::TimerEvent &e) {
  static int delay = 0;
  if (++delay < 5)
    return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();

    Position update_bbox_min, update_bbox_max;
    ft->getUpdateBBox(update_bbox_min, update_bbox_max);
    ed->update_bbox_min_ = update_bbox_min;
    ed->update_bbox_max_ = update_bbox_max;

    ft->getFrontiers(ed->frontiers_);
    ft->getDormantFrontiers(ed->dormant_frontiers_);
    ft->getTinyFrontiers(ed->tiny_frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);
    ft->getTopViewpointsInfo(fd_->odom_pos_, ed->points_, ed->yaws_, ed->averages_);

    double map_resolution = planner_manager_->map_server_->getResolution();
    if (map_resolution > 0.1) {
      visualization_->drawFrontierPointcloudHighResolution(ed->frontiers_, map_resolution);
      visualization_->drawDormantFrontierPointcloudHighResolution(ed->dormant_frontiers_,
                                                                  map_resolution);
      visualization_->drawTinyFrontierPointcloudHighResolution(ed->tiny_frontiers_, map_resolution);
    } else {
      visualization_->drawFrontierPointcloud(ed->frontiers_);
      visualization_->drawDormantFrontierPointcloud(ed->dormant_frontiers_);
      visualization_->drawTinyFrontierPointcloud(ed->tiny_frontiers_);
    }

    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawText(ed->averages_[i], to_string(i), 0.5,
                               PlanningVisualization::Color::Red(), "frontier_id", i,
                               PlanningVisualization::PUBLISHER::FRONTIER);
      visualization_->drawSpheres(ed->points_, 0.2, PlanningVisualization::Color::DeepGreen(),
                                  "points", 0, PlanningVisualization::PUBLISHER::VIEWPOINT);
    }

    if (ed->frontiers_.size() > 0) {
      frontier_ready_ = true;
    }
  }

  if (expl_manager_->ep_->auto_start_ && frontier_ready_ && state_ == WAIT_TRIGGER) {
    fd_->triggered_ = true;
    transitState(PLAN_TRAJ, "frontierCallback");

    expl_manager_->ed_->start_time_ = ros::Time::now();
    ROS_INFO("[FSM] Exploration start time: %lf", expl_manager_->ed_->start_time_.toSec());
  }
}

void ExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg) {
  if (state_ != WAIT_TRIGGER)
    return;

  // Can be only triggered after frontier is generated
  if (!frontier_ready_)
    return;

  fd_->triggered_ = true;
  transitState(PLAN_TRAJ, "triggerCallback");

  expl_manager_->ed_->start_time_ = ros::Time::now();
  ROS_INFO("[FSM] Exploration start time: %lf", expl_manager_->ed_->start_time_.toSec());
}

void ExplorationFSM::safetyCallback(const ros::TimerEvent &e) {
  // Check if current pos is in occupied area
  // CHECK(planner_manager_->map_server_->getOccupancy(fd_->odom_pos_) !=
  //       voxel_mapping::OccupancyType::OCCUPIED)
  //     << "Exploration failed as current pos is in occupied area. Please restart the program.";

  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    bool safe = planner_manager_->checkTrajCollision();
    if (!safe) {
      ROS_ERROR("[FSM] Collision detected on the trajectory! Replanning...");
      std_msgs::Int32 replan_msg;
      replan_msg.data = 1;
      replan_pub_.publish(replan_msg);
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void ExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = ros::Time::now();
  }

  if (state_ == INIT || state_ == WAIT_TRIGGER)
    return;

  static int delay = 0;
  static double accum_dist = 0;
  if (++delay < 10)
    return;
  else {
    delay = 0;
    if (expl_manager_->ed_->trajecotry_.size() > 0)
      accum_dist += (fd_->odom_pos_ - expl_manager_->ed_->trajecotry_.back()).norm();
    expl_manager_->ed_->trajecotry_.push_back(fd_->odom_pos_);
  }
}

void ExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  ROS_WARN_STREAM("[FSM] "
                  << "Transit state from " + fd_->state_str_[pre_s] + " to " +
                         fd_->state_str_[int(new_state)] + " by " + pos_call);
  LOG(WARNING) << "[FSM] "
               << "ROS Time: " << ros::Time::now()
               << " Transit state from " + fd_->state_str_[pre_s] + " to " +
                      fd_->state_str_[int(new_state)] + " by " + pos_call;
}
} // namespace fast_planner
