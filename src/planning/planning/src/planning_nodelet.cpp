#include <geometry_msgs/PoseStamped.h>
#include <mapping/mapping.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/ReplanState.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>

#include <Eigen/Core>
#include <env/env.hpp>
#include <prediction/prediction.hpp>
#include <thread>
#include <visualization/visualization.hpp>
#include <wr_msg/wr_msg.hpp>

#include "kinematic/kinematic.hpp"

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ",
                             ", ", "", "", " << ", ";");

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber gridmap_sub_, odom_sub_, target_sub_, triger_sub_,
      land_triger_sub_;
  ros::Timer plan_timer_;

  ros::Publisher traj_pub_, heartbeat_pub_, replanState_pub_;

  std::shared_ptr<mapping::OccGridMap> gridmapPtr_;
  std::shared_ptr<env::Env> envPtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
  std::shared_ptr<prediction::Predict> prePtr_;

  // NOTE planning or fake target
  bool fake_ = false;
  double relative_height_ = 2.0;
  Eigen::Vector3d goal_;
  Eigen::Vector3d land_p_;
  Eigen::Quaterniond land_q_;

  // NOTE just for debug
  bool debug_ = false;
  quadrotor_msgs::ReplanState replanStateMsg_;
  ros::Publisher gridmap_pub_, inflate_gridmap_pub_;
  quadrotor_msgs::OccMap3d occmap_msg_;

  double tracking_dur_, tracking_dist_, tolerance_d_;

  Trajectory traj_poly_;
  ros::Time replan_stamp_;
  Kinematic::Control controller_;

  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;

  nav_msgs::Odometry odom_msg_, target_msg_;
  quadrotor_msgs::OccMap3d map_msg_;
  std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag gridmap_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool map_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool target_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool land_triger_received_ = ATOMIC_VAR_INIT(false);

  void pub_hover_p(const Eigen::Vector3d& hover_p, const ros::Time& stamp) {
    ROS_INFO("WE ARE IN PUB HOBER P");
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = true;
    traj_msg.hover_p.resize(3);
    for (int i = 0; i < 3; ++i) {
      traj_msg.hover_p[i] = hover_p[i];
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_pub_.publish(traj_msg);
  }
  void pub_traj(const Trajectory& traj, const double& yaw,
                const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = false;
    traj_msg.order = 5;
    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize(6 * piece_num);
    traj_msg.coef_y.resize(6 * piece_num);
    traj_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i) {
      traj_msg.duration[i] = durs(i);
      CoefficientMat cMat = traj[i].getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++) {
        traj_msg.coef_x[i6 + j] = cMat(0, j);
        traj_msg.coef_y[i6 + j] = cMat(1, j);
        traj_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    // NOTE yaw
    traj_msg.yaw = yaw;
    traj_pub_.publish(traj_msg);
  }

  // 设置起点
  // 给一个0 0 0.9,现在已经删除了
  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 0.9;
    triger_received_ = true;
  }

  // 降落回调函数
  void land_triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    land_p_.x() = msgPtr->pose.position.x;
    land_p_.y() = msgPtr->pose.position.y;
    land_p_.z() = msgPtr->pose.position.z;
    land_q_.w() = msgPtr->pose.orientation.w;
    land_q_.x() = msgPtr->pose.orientation.x;
    land_q_.y() = msgPtr->pose.orientation.y;
    land_q_.z() = msgPtr->pose.orientation.z;
    land_triger_received_ = true;
  }

  // ego的定位信息，需要从airsim去更新
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (odom_lock_.test_and_set());
    odom_msg_ = *msgPtr;
    odom_received_ = true;
    odom_lock_.clear();
  }

  void airsim_odom_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (odom_lock_.test_and_set());
    auto tmp_msg = *msgPtr;
    odom_msg_ = tmp_msg;
    // odom_msg_.pose.postion.z = -tmp_msg.pose.postion.z;
    // odom_msg_.pose.postion.x = tmp_msg.pose.postion.y;
    // odom_msg_.pose.postion.y = tmp_msg.pose.postion.x;
    // ROS_INFO("Position: (%f, %f, %f)", msgPtr->pose.pose.position.x,
    // msgPtr->pose.pose.position.y, msgPtr->pose.pose.position.z);
    odom_received_ = true;
    odom_lock_.clear();
  }

  void target_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (target_lock_.test_and_set());
    target_msg_ = *msgPtr;
    target_received_ = true;
    target_lock_.clear();
  }

  void gridmap_callback(const quadrotor_msgs::OccMap3dConstPtr& msgPtr) {
    while (gridmap_lock_.test_and_set());
    map_msg_ = *msgPtr;
    map_received_ = true;
    gridmap_lock_.clear();
  }

  //  关于kinematic控制
  void kinematic_control_callback(const ros::TimerEvent& event) {
    ROS_INFO(
        "-------------------------now we are in kinematic "
        "control-----------------------------");
    ros::Time start_time = ros::Time::now();
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!odom_received_ || !map_received_) {
      return;
    }
    // obtain state of odom
    while (odom_lock_.test_and_set());
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    // odom_p、odom_v、odom_q是ego的位置、速度和四元数信息
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    ROS_INFO("in planning odom (x,y,z) = (%f,%f,%f)", odom_p.x(), odom_p.y(),
             odom_p.z());
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    Eigen::Quaterniond odom_q(
        odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);
    // 这个是卡尔曼滤波成功的标志
    ROS_INFO("target_received_ = %s", target_received_ ? "true" : "false");
    if (!target_received_) {
      return;
    }
    ROS_INFO("force_hover_ = %s", force_hover_ ? "true" : "false");
    // NOTE obtain state of target
    // 只要target是锁的状态就不动
    while (target_lock_.test_and_set());
    replanStateMsg_.target = target_msg_;
    target_lock_.clear();
    // target_p、target_v、target_q是目标物体的信息
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    Eigen::Quaterniond target_q;
    target_q.w() = replanStateMsg_.target.pose.pose.orientation.w;
    target_q.x() = replanStateMsg_.target.pose.pose.orientation.x;
    target_q.y() = replanStateMsg_.target.pose.pose.orientation.y;
    target_q.z() = replanStateMsg_.target.pose.pose.orientation.z;

    // NOTE force-hover: waiting for the speed of drone small enough
    ROS_INFO("odom_v.norm() = %f", odom_v.norm());
    // if (force_hover_ && odom_v.norm() > 0.1) {
    //   return;
    // }
    // NOTE just for landing on the car!
    // 这段是着陆使用的，目前没有被用到
    if (land_triger_received_) {
      if (std::fabs((target_p - odom_p).norm() < 0.1 && odom_v.norm() < 0.1 &&
                    target_v.norm() < 0.2)) {
        if (!wait_hover_) {
          pub_hover_p(odom_p, ros::Time::now());
          wait_hover_ = true;
        }
        ROS_WARN("[planner] HOVERING...");
        return;
      }
      // TODO get the orientation fo target and calculate the pose of landing
      // point
      target_p = target_p + target_q * land_p_;
      wait_hover_ = false;
    } else {
      // todo 0812 定高飞行，记得修改，原来是1.0
      target_p.z() += relative_height_;
      // NOTE determin whether to replan
      // 计算当前位置和目标位置之间的差
      Eigen::Vector3d dp = target_p - odom_p;
      // TODO FXJ 在实际的飞机上需要修改!!!!
      double desired_yaw = std::atan2(dp.y(), dp.x()) - M_PI / 2.0;
      ROS_INFO("desired_yaw = %f", desired_yaw);
      Eigen::Vector3d project_yaw =
          odom_q.toRotationMatrix().col(0);  // NOTE ZYX
      double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());

      ROS_INFO("now_yaw = %f", now_yaw);
      if (std::fabs((target_p - odom_p).norm() - tracking_dist_) <
              tolerance_d_ &&
          odom_v.norm() < 0.1 && target_v.norm() < 0.2 &&
          std::fabs(desired_yaw - now_yaw) < 0.5) {
        if (!wait_hover_) {
          pub_hover_p(odom_p, ros::Time::now());
          wait_hover_ = true;
        }
        ROS_WARN("[planner] HOVERING...");
        replanStateMsg_.state = -1;
        replanState_pub_.publish(replanStateMsg_);
        return;
      } else {
        wait_hover_ = false;
      }
    }

    // NOTE obtain map
    while (gridmap_lock_.test_and_set());
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();
    prePtr_->setMap(*gridmapPtr_);

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(odom_p, target_p)) {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);
    } else {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);
    }

    // *********************生成目标物体预测轨迹的部分*********************
    std::vector<Eigen::Vector3d> target_predcit;
    ros::Time t_start = ros::Time::now();
    bool generate_new_traj_success =
        prePtr_->predict(target_p, target_v, target_predcit);
    ros::Time t_stop = ros::Time::now();
    double cost_time = (t_stop - t_start).toSec() * 1e3;
    ROS_INFO("predict cost time: %f ms", cost_time);
    ROS_INFO("generate predict trajectory = %s",
             generate_new_traj_success ? "true" : "false");
    // *********************生成目标物体预测轨迹的部分*********************
    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      // 这儿的目的是显示一个目标位置的圈，这个圈是自己必须到达目标物体俯视图的范围
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(
            observable_p +
            tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // *********************设置初始状态的部分*********************
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
    double replan_t = (replan_stamp - replan_stamp_).toSec();
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
      // 如果不正常状态的话就从上一帧规划的轨迹上拿参数
      ROS_INFO("Ntraj_poly_.getTotalDuration() = %f",
               traj_poly_.getTotalDuration());
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      ROS_INFO("normal state, replan_t = %f", replan_t);
      // should replan from the last trajectory
      // 如果正常状态的话就从上一帧规划的轨迹上递推得到初始状态点
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) =
        iniState;
    // *********************设置初始状态的部分*********************

    // *********************利用A*生成初始轨迹的部分*********************
    Eigen::Vector3d p_start = iniState.col(0);
    ROS_INFO("Initial State Matrix:");
    for (int i = 0; i < iniState.rows(); ++i) {
      std::ostringstream oss;
      for (int j = 0; j < iniState.cols(); ++j) {
        oss << iniState(i, j) << ", ";
      }
      ROS_INFO("%s", oss.str().c_str());
    }
    std::vector<Eigen::Vector3d> path, way_pts;

    // NOTE calculate time of path searching, corridor generation and
    // optimization

    // static double t_path_ = 0;
    // static double t_corridor_ = 0;
    // static double t_optimization_ = 0;
    // static int times_path_ = 0;
    // static int times_corridor_ = 0;
    // static int times_optimization_ = 0;
    // double t_path = 0;

    if (generate_new_traj_success) {
      // ros::Time t_front0 = ros::Time::now();
      if (land_triger_received_) {
        generate_new_traj_success =
            envPtr_->short_astar(p_start, target_p, path);
      } else {
        // A*的入口
        generate_new_traj_success =
            envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
        ROS_INFO("give A STAR start point (x, y, z) = (%f, %f, %f)",
                 p_start.x(), p_start.y(), p_start.z());
        ROS_INFO("target_predcit.size() = %lu", target_predcit.size());
        ROS_INFO("way_pts.size() = %lu", way_pts.size());
        ROS_INFO("path.size() = %lu", path.size());
      }
      // ros::Time t_end0 = ros::Time::now();
    }
    ROS_INFO("generate A* trajectory = %s",
             generate_new_traj_success ? "true" : "false");
    // *********************利用A*生成初始轨迹的部分*********************

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      if (land_triger_received_) {
        for (const auto& p : target_predcit) {
          path.push_back(p);
        }
      } else {
        // *********************生成扇形可视化区域并且生成种子*********************
        // NOTE generate visible regions
        target_predcit.pop_back();
        way_pts.pop_back();
        ROS_INFO("in visible regions target_predcit.size() = %lu",
                 target_predcit.size());
        ROS_INFO("in visible regions way_pts.size() = %lu", way_pts.size());
        // ros::Time t_front1 = ros::Time::now();
        envPtr_->generate_visible_regions(target_predcit, way_pts, visible_ps,
                                          thetas);
        // 把way_pts作处理得到seed，作为下面的输入，这个seed是根据可视区域得到的位于左右区域上的一个点，
        // 他的确定原则是基于原来的点位于的区域，而后旋转一定角度得到的。
        // thetas是左右的角度最大值，visible_ps是角平分线上的距离为允许距离的点
        // ros::Time t_end1 = ros::Time::now();
        // t_path += (t_end1 - t_front1).toSec() * 1e3;
        visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
        visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas,
                                            "visible_region");

        // TODO change the final state
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
        for (int i = 0; i < (int)way_pts.size(); ++i) {
          rays.emplace_back(target_predcit[i], way_pts[i]);
        }
        visPtr_->visualize_pointcloud(way_pts, "way_pts");
        way_pts.insert(way_pts.begin(), p_start);
        // ros::Time t_front2 = ros::Time::now();
        // 把点转化成路径，确保点不碰撞在障碍范围内一定范围内。
        envPtr_->pts2path(way_pts, path);
        // ros::Time t_end2 = ros::Time::now();
        // t_path += (t_end2 - t_front2).toSec() * 1e3;
      }
      // *********************生成扇形可视化区域并且生成种子*********************
      // NOTE corridor generating
      // *********************生成飞行走廊*********************
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;

      // ros::Time t_front3 = ros::Time::now();
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      for (size_t i = 0; i < path.size(); ++i) {
        const Eigen::Vector3d& vec = path[i];
        ROS_INFO("A STAR path[%zu]: [%f, %f, %f]", i, vec.x(), vec.y(),
                 vec.z());
      }
      ROS_INFO("hPolys.size() = %lu", hPolys.size());
      ROS_INFO("keyPts.size() = %lu", keyPts.size());
      for (size_t i = 0; i < keyPts.size(); ++i) {
        const Eigen::Vector3d& first = keyPts[i].first;
        const Eigen::Vector3d& second = keyPts[i].second;
        ROS_INFO("keyPts[%zu] - First Vector: [%f, %f, %f]", i, first.x(),
                 first.y(), first.z());
        ROS_INFO("keyPts[%zu] - Second Vector: [%f, %f, %f]", i, second.x(),
                 second.y(), second.z());
      }
      // ros::Time t_end3 = ros::Time::now();
      // double t_corridor = (t_end3 - t_front3).toSec() * 1e3;

      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");
      // *********************生成飞行走廊*********************

      // *********************轨迹优化部分*********************
      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      // 末端状态
      finState.col(0) = path.back();
      // finState.col(1) = target_v;
      finState.col(1) = -1.0 * target_v;
      ROS_INFO("Final State Matrix:");
      for (int i = 0; i < finState.rows(); ++i) {
        std::ostringstream oss;
        for (int j = 0; j < finState.cols(); ++j) {
          oss << finState(i, j) << ", ";
        }
        ROS_INFO("%s", oss.str().c_str());
      }
      // ros::Time t_front4 = ros::Time::now();

      // 这个没有被用到
      if (land_triger_received_) {
        finState.col(0) = target_predcit.back();
        generate_new_traj_success = trajOptPtr_->generate_traj(
            iniState, finState, target_predcit, hPolys, traj);
      } else {
        generate_new_traj_success =
            trajOptPtr_->generate_traj(iniState, finState, target_predcit,
                                       visible_ps, thetas, hPolys, traj);
      }
      // *********************轨迹优化部分*********************
      // 获取结束时间
      ros::Time end_time = ros::Time::now();
      // 计算并输出执行时间
      ros::Duration execution_time = end_time - start_time;
      ROS_INFO("plan_timer_callback execution time = %f s",
               execution_time.toSec());
      // ros::Time t_end4 = ros::Time::now();
      // double t_optimization = (t_end4 - t_front4).toSec() * 1e3;

      // NOTE average calculating time of path searching, corridor generation
      // and optimization

      // t_path_ = (t_path_ * times_path_ + t_path) / (++times_path_);
      // t_corridor_ = (t_corridor_ * times_corridor_ + t_corridor) /
      // (++times_corridor_); t_optimization_ = (t_optimization_ *
      // times_optimization_ + t_optimization) / (++times_optimization_);

      // std::cout << "t_path_: " << t_path_ << " ms" << std::endl;
      // std::cout << "t_corridor_: " << t_corridor_ << " ms" << std::endl;
      // std::cout << "t_optimization_: " << t_optimization_ << " ms" <<
      // std::endl;

      visPtr_->visualize_traj(traj, "traj");
    }
    ROS_INFO("generate optimize trajectory = %s",
             generate_new_traj_success ? "true" : "false");
    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      ROS_WARN("[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      Eigen::Vector3d dp = target_p + target_v * 0.03 - iniState.col(0);
      // NOTE : if the drone is going to unknown areas, watch that direction
      // Eigen::Vector3d un_known_p = traj.getPos(1.0);
      // if (gridmapPtr_->isUnKnown(un_known_p)) {
      //   dp = un_known_p - odom_p;
      // }
      double yaw = std::atan2(dp.y(), dp.x());
      if (land_triger_received_) {
        yaw = 2 * std::atan2(target_q.z(), target_q.w());
      }
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  // NOTE main callback 无人机用到的回调函数
  void plan_timer_callback(const ros::TimerEvent& event) {
    ROS_INFO(
        "----------------------------------------------------------------------"
        "----------------------------");
    // 获取开始时间
    ros::Time start_time = ros::Time::now();
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!odom_received_ || !map_received_) {
      return;
    }
    // obtain state of odom
    while (odom_lock_.test_and_set());
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    // odom_p、odom_v、odom_q是ego的位置、速度和四元数信息
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    ROS_INFO("in planning odom (x,y,z) = (%f,%f,%f)", odom_p.x(), odom_p.y(),
             odom_p.z());
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    Eigen::Quaterniond odom_q(
        odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);
    // if (!triger_received_) {
    //   return;
    // }
    // 这个是卡尔曼滤波成功的标志
    ROS_INFO("target_received_ = %s", target_received_ ? "true" : "false");
    if (!target_received_) {
      return;
    }
    ROS_INFO("force_hover_ = %s", force_hover_ ? "true" : "false");
    // NOTE obtain state of target
    // 只要target是锁的状态就不动
    while (target_lock_.test_and_set());
    replanStateMsg_.target = target_msg_;
    target_lock_.clear();
    // target_p、target_v、target_q是目标物体的信息
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    Eigen::Quaterniond target_q;
    target_q.w() = replanStateMsg_.target.pose.pose.orientation.w;
    target_q.x() = replanStateMsg_.target.pose.pose.orientation.x;
    target_q.y() = replanStateMsg_.target.pose.pose.orientation.y;
    target_q.z() = replanStateMsg_.target.pose.pose.orientation.z;

    // NOTE force-hover: waiting for the speed of drone small enough
    ROS_INFO("odom_v.norm() = %f", odom_v.norm());
    // if (force_hover_ && odom_v.norm() > 0.1) {
    //   return;
    // }
    // NOTE just for landing on the car!
    // 这段是着陆使用的，目前没有被用到
    if (land_triger_received_) {
      if (std::fabs((target_p - odom_p).norm() < 0.1 && odom_v.norm() < 0.1 &&
                    target_v.norm() < 0.2)) {
        if (!wait_hover_) {
          pub_hover_p(odom_p, ros::Time::now());
          wait_hover_ = true;
        }
        ROS_WARN("[planner] HOVERING...");
        return;
      }
      // TODO get the orientation fo target and calculate the pose of landing
      // point
      target_p = target_p + target_q * land_p_;
      wait_hover_ = false;
    } else {
      // todo 0812 定高飞行，记得修改，原来是1.0
      target_p.z() += relative_height_;
      // NOTE determin whether to replan
      // 计算当前位置和目标位置之间的差
      Eigen::Vector3d dp = target_p - odom_p;
      // TODO FXJ 在实际的飞机上需要修改!!!!
      double desired_yaw = std::atan2(dp.y(), dp.x()) - M_PI / 2.0;
      ROS_INFO("desired_yaw = %f", desired_yaw);
      Eigen::Vector3d project_yaw =
          odom_q.toRotationMatrix().col(0);  // NOTE ZYX
      double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());

      ROS_INFO("now_yaw = %f", now_yaw);
      if (std::fabs((target_p - odom_p).norm() - tracking_dist_) <
              tolerance_d_ &&
          odom_v.norm() < 0.1 && target_v.norm() < 0.2 &&
          std::fabs(desired_yaw - now_yaw) < 0.5) {
        if (!wait_hover_) {
          pub_hover_p(odom_p, ros::Time::now());
          wait_hover_ = true;
        }
        ROS_WARN("[planner] HOVERING...");
        replanStateMsg_.state = -1;
        replanState_pub_.publish(replanStateMsg_);
        return;
      } else {
        wait_hover_ = false;
      }
    }

    // NOTE obtain map
    while (gridmap_lock_.test_and_set());
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();
    prePtr_->setMap(*gridmapPtr_);

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(odom_p, target_p)) {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);
    } else {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);
    }

    // *********************生成目标物体预测轨迹的部分*********************
    std::vector<Eigen::Vector3d> target_predcit;
    ros::Time t_start = ros::Time::now();
    bool generate_new_traj_success =
        prePtr_->predict(target_p, target_v, target_predcit);
    ros::Time t_stop = ros::Time::now();
    double cost_time = (t_stop - t_start).toSec() * 1e3;
    ROS_INFO("predict cost time: %f ms", cost_time);
    ROS_INFO("generate predict trajectory = %s",
             generate_new_traj_success ? "true" : "false");
    // *********************生成目标物体预测轨迹的部分*********************
    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      // 这儿的目的是显示一个目标位置的圈，这个圈是自己必须到达目标物体俯视图的范围
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(
            observable_p +
            tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // *********************设置初始状态的部分*********************
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
    double replan_t = (replan_stamp - replan_stamp_).toSec();
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
      // 如果不正常状态的话就从上一帧规划的轨迹上拿参数
      ROS_INFO("Ntraj_poly_.getTotalDuration() = %f",
               traj_poly_.getTotalDuration());
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      ROS_INFO("normal state, replan_t = %f", replan_t);
      // should replan from the last trajectory
      // 如果正常状态的话就从上一帧规划的轨迹上递推得到初始状态点
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) =
        iniState;
    // *********************设置初始状态的部分*********************

    // *********************利用A*生成初始轨迹的部分*********************
    Eigen::Vector3d p_start = iniState.col(0);
    ROS_INFO("Initial State Matrix:");
    for (int i = 0; i < iniState.rows(); ++i) {
      std::ostringstream oss;
      for (int j = 0; j < iniState.cols(); ++j) {
        oss << iniState(i, j) << ", ";
      }
      ROS_INFO("%s", oss.str().c_str());
    }
    std::vector<Eigen::Vector3d> path, way_pts;

    // NOTE calculate time of path searching, corridor generation and
    // optimization

    // static double t_path_ = 0;
    // static double t_corridor_ = 0;
    // static double t_optimization_ = 0;
    // static int times_path_ = 0;
    // static int times_corridor_ = 0;
    // static int times_optimization_ = 0;
    // double t_path = 0;

    if (generate_new_traj_success) {
      // ros::Time t_front0 = ros::Time::now();
      if (land_triger_received_) {
        generate_new_traj_success =
            envPtr_->short_astar(p_start, target_p, path);
      } else {
        // A*的入口
        generate_new_traj_success =
            envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
        ROS_INFO("give A STAR start point (x, y, z) = (%f, %f, %f)",
                 p_start.x(), p_start.y(), p_start.z());
        ROS_INFO("target_predcit.size() = %lu", target_predcit.size());
        ROS_INFO("way_pts.size() = %lu", way_pts.size());
        ROS_INFO("path.size() = %lu", path.size());
      }
      // ros::Time t_end0 = ros::Time::now();
    }
    ROS_INFO("generate A* trajectory = %s",
             generate_new_traj_success ? "true" : "false");
    // *********************利用A*生成初始轨迹的部分*********************

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      if (land_triger_received_) {
        for (const auto& p : target_predcit) {
          path.push_back(p);
        }
      } else {
        // *********************生成扇形可视化区域并且生成种子*********************
        // NOTE generate visible regions
        target_predcit.pop_back();
        way_pts.pop_back();
        ROS_INFO("in visible regions target_predcit.size() = %lu",
                 target_predcit.size());
        ROS_INFO("in visible regions way_pts.size() = %lu", way_pts.size());
        // ros::Time t_front1 = ros::Time::now();
        envPtr_->generate_visible_regions(target_predcit, way_pts, visible_ps,
                                          thetas);
        // 把way_pts作处理得到seed，作为下面的输入，这个seed是根据可视区域得到的位于左右区域上的一个点，
        // 他的确定原则是基于原来的点位于的区域，而后旋转一定角度得到的。
        // thetas是左右的角度最大值，visible_ps是角平分线上的距离为允许距离的点
        // ros::Time t_end1 = ros::Time::now();
        // t_path += (t_end1 - t_front1).toSec() * 1e3;
        visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
        visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas,
                                            "visible_region");

        // TODO change the final state
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
        for (int i = 0; i < (int)way_pts.size(); ++i) {
          rays.emplace_back(target_predcit[i], way_pts[i]);
        }
        visPtr_->visualize_pointcloud(way_pts, "way_pts");
        way_pts.insert(way_pts.begin(), p_start);
        // ros::Time t_front2 = ros::Time::now();
        // 把点转化成路径，确保点不碰撞在障碍范围内一定范围内。
        envPtr_->pts2path(way_pts, path);
        // ros::Time t_end2 = ros::Time::now();
        // t_path += (t_end2 - t_front2).toSec() * 1e3;
      }
      // *********************生成扇形可视化区域并且生成种子*********************
      // NOTE corridor generating
      // *********************生成飞行走廊*********************
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;

      // ros::Time t_front3 = ros::Time::now();
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      for (size_t i = 0; i < path.size(); ++i) {
        const Eigen::Vector3d& vec = path[i];
        ROS_INFO("A STAR path[%zu]: [%f, %f, %f]", i, vec.x(), vec.y(),
                 vec.z());
      }
      ROS_INFO("hPolys.size() = %lu", hPolys.size());
      ROS_INFO("keyPts.size() = %lu", keyPts.size());
      for (size_t i = 0; i < keyPts.size(); ++i) {
        const Eigen::Vector3d& first = keyPts[i].first;
        const Eigen::Vector3d& second = keyPts[i].second;
        ROS_INFO("keyPts[%zu] - First Vector: [%f, %f, %f]", i, first.x(),
                 first.y(), first.z());
        ROS_INFO("keyPts[%zu] - Second Vector: [%f, %f, %f]", i, second.x(),
                 second.y(), second.z());
      }
      // ros::Time t_end3 = ros::Time::now();
      // double t_corridor = (t_end3 - t_front3).toSec() * 1e3;

      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");
      // *********************生成飞行走廊*********************

      // *********************轨迹优化部分*********************
      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      // 末端状态
      finState.col(0) = path.back();
      // finState.col(1) = target_v;
      finState.col(1) = -1.0 * target_v;
      ROS_INFO("Final State Matrix:");
      for (int i = 0; i < finState.rows(); ++i) {
        std::ostringstream oss;
        for (int j = 0; j < finState.cols(); ++j) {
          oss << finState(i, j) << ", ";
        }
        ROS_INFO("%s", oss.str().c_str());
      }
      // ros::Time t_front4 = ros::Time::now();

      // 这个没有被用到
      if (land_triger_received_) {
        finState.col(0) = target_predcit.back();
        generate_new_traj_success = trajOptPtr_->generate_traj(
            iniState, finState, target_predcit, hPolys, traj);
      } else {
        generate_new_traj_success =
            trajOptPtr_->generate_traj(iniState, finState, target_predcit,
                                       visible_ps, thetas, hPolys, traj);
      }
      // *********************轨迹优化部分*********************
      // 获取结束时间
      ros::Time end_time = ros::Time::now();
      // 计算并输出执行时间
      ros::Duration execution_time = end_time - start_time;
      ROS_INFO("plan_timer_callback execution time = %f s",
               execution_time.toSec());
      // ros::Time t_end4 = ros::Time::now();
      // double t_optimization = (t_end4 - t_front4).toSec() * 1e3;

      // NOTE average calculating time of path searching, corridor generation
      // and optimization

      // t_path_ = (t_path_ * times_path_ + t_path) / (++times_path_);
      // t_corridor_ = (t_corridor_ * times_corridor_ + t_corridor) /
      // (++times_corridor_); t_optimization_ = (t_optimization_ *
      // times_optimization_ + t_optimization) / (++times_optimization_);

      // std::cout << "t_path_: " << t_path_ << " ms" << std::endl;
      // std::cout << "t_corridor_: " << t_corridor_ << " ms" << std::endl;
      // std::cout << "t_optimization_: " << t_optimization_ << " ms" <<
      // std::endl;

      visPtr_->visualize_traj(traj, "traj");
    }
    ROS_INFO("generate optimize trajectory = %s",
             generate_new_traj_success ? "true" : "false");
    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      ROS_WARN("[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      Eigen::Vector3d dp = target_p + target_v * 0.03 - iniState.col(0);
      // NOTE : if the drone is going to unknown areas, watch that direction
      // Eigen::Vector3d un_known_p = traj.getPos(1.0);
      // if (gridmapPtr_->isUnKnown(un_known_p)) {
      //   dp = un_known_p - odom_p;
      // }
      double yaw = std::atan2(dp.y(), dp.x());
      if (land_triger_received_) {
        yaw = 2 * std::atan2(target_q.z(), target_q.w());
      }
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  // 这个函数没有被用到
  void airsim_fake_timer_callback(const ros::TimerEvent& event) {
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!odom_received_ || !map_received_) {
      return;
    }
    // obtain state of odom
    while (odom_lock_.test_and_set());
    // 拿到ego的定位信息
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    if (!triger_received_) {
      return;
    }
    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      return;
    }
    // NOTE local goal
    Eigen::Vector3d local_goal;
    Eigen::Vector3d delta = goal_ - odom_p;
    std::cout << "delta.norm() = " << delta.norm() << std::endl;
    if (delta.norm() < 15) {
      local_goal = goal_;
    } else {
      std::cout << "else if (delta.norm() < 15)" << delta.norm() << std::endl;
      local_goal = delta.normalized() * 15 + odom_p;
    }
    // 打印 odom_p 的值
    std::cout << "Odom position: "
              << "x: " << odom_p.x() << ", "
              << "y: " << odom_p.y() << ", "
              << "z: " << odom_p.z() << std::endl;
    // NOTE obtain map
    while (gridmap_lock_.test_and_set());
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();

    // NOTE determin whether to replan
    bool no_need_replan = false;
    if (!force_hover_ && !wait_hover_) {
      double last_traj_t_rest = traj_poly_.getTotalDuration() -
                                (ros::Time::now() - replan_stamp_).toSec();
      bool new_goal =
          (local_goal - traj_poly_.getPos(traj_poly_.getTotalDuration()))
              .norm() > tracking_dist_;
      if (!new_goal) {
        if (last_traj_t_rest < 1.0) {
          ROS_WARN("[planner] NEAR GOAL...");
          no_need_replan = true;
        } else if (validcheck(traj_poly_, replan_stamp_, last_traj_t_rest)) {
          ROS_WARN("[planner] NO NEED REPLAN...");
          double t_delta = traj_poly_.getTotalDuration() < 1.0
                               ? traj_poly_.getTotalDuration()
                               : 1.0;
          double t_yaw = (ros::Time::now() - replan_stamp_).toSec() + t_delta;
          Eigen::Vector3d un_known_p = traj_poly_.getPos(t_yaw);
          Eigen::Vector3d dp = un_known_p - odom_p;
          double yaw = std::atan2(dp.y(), dp.x());
          pub_traj(traj_poly_, yaw, replan_stamp_);
          no_need_replan = true;
        }
      }
    }
    // NOTE determin whether to pub hover
    if ((goal_ - odom_p).norm() < tracking_dist_ + tolerance_d_ &&
        odom_v.norm() < 0.1) {
      if (!wait_hover_) {
        pub_hover_p(odom_p, ros::Time::now());
        wait_hover_ = true;
      }
      ROS_WARN("[planner] HOVERING...");
      replanStateMsg_.state = -1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else {
      wait_hover_ = false;
    }
    if (no_need_replan) {
      return;
    }

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
    double replan_t = (replan_stamp - replan_stamp_).toSec();
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      // should replan from the last trajectory
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) =
        iniState;

    // NOTE generate an extra corridor
    Eigen::Vector3d p_start = iniState.col(0);
    bool need_extra_corridor = iniState.col(1).norm() > 1.0;
    Eigen::MatrixXd hPoly;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> line;
    if (need_extra_corridor) {
      Eigen::Vector3d v_norm = iniState.col(1).normalized();
      line.first = p_start;
      double step = 0.1;
      for (double dx = step; dx < 1.0; dx += step) {
        p_start += step * v_norm;
        if (gridmapPtr_->isOccupied(p_start)) {
          p_start -= step * v_norm;
          break;
        }
      }
      line.second = p_start;
      envPtr_->generateOneCorridor(line, 2.0, hPoly);
    }
    // NOTE path searching
    std::vector<Eigen::Vector3d> path;
    bool generate_new_traj_success =
        envPtr_->astar_search(p_start, local_goal, path);
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      if (need_extra_corridor) {
        hPolys.insert(hPolys.begin(), hPoly);
        keyPts.insert(keyPts.begin(), line);
      }
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      // return;
      generate_new_traj_success =
          trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);
      visPtr_->visualize_traj(traj, "traj");
    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      ROS_WARN("[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      // NOTE : if the trajectory is known, watch that direction
      Eigen::Vector3d un_known_p = traj.getPos(
          traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
      Eigen::Vector3d dp = un_known_p - odom_p;
      double yaw = std::atan2(dp.y(), dp.x());
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (!validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  // 目标物体实际用到的时间回调函数
  void fake_timer_callback(const ros::TimerEvent& event) {
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!odom_received_ || !map_received_) {
      return;
    }
    // obtain state of odom
    while (odom_lock_.test_and_set());
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    if (!triger_received_) {
      return;
    }
    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      return;
    }

    // NOTE local goal
    Eigen::Vector3d local_goal;
    Eigen::Vector3d delta = goal_ - odom_p;
    if (delta.norm() < 15) {
      local_goal = goal_;
    } else {
      local_goal = delta.normalized() * 15 + odom_p;
    }

    // NOTE obtain map
    while (gridmap_lock_.test_and_set());
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();

    // NOTE determin whether to replan
    bool no_need_replan = false;
    if (!force_hover_ && !wait_hover_) {
      double last_traj_t_rest = traj_poly_.getTotalDuration() -
                                (ros::Time::now() - replan_stamp_).toSec();
      bool new_goal =
          (local_goal - traj_poly_.getPos(traj_poly_.getTotalDuration()))
              .norm() > tracking_dist_;
      if (!new_goal) {
        if (last_traj_t_rest < 1.0) {
          ROS_WARN("[planner] NEAR GOAL...");
          no_need_replan = true;
        } else if (validcheck(traj_poly_, replan_stamp_, last_traj_t_rest)) {
          ROS_WARN("[planner] NO NEED REPLAN...");
          double t_delta = traj_poly_.getTotalDuration() < 1.0
                               ? traj_poly_.getTotalDuration()
                               : 1.0;
          double t_yaw = (ros::Time::now() - replan_stamp_).toSec() + t_delta;
          Eigen::Vector3d un_known_p = traj_poly_.getPos(t_yaw);
          Eigen::Vector3d dp = un_known_p - odom_p;
          double yaw = std::atan2(dp.y(), dp.x());
          pub_traj(traj_poly_, yaw, replan_stamp_);
          no_need_replan = true;
        }
      }
    }
    // NOTE determin whether to pub hover
    if ((goal_ - odom_p).norm() < tracking_dist_ + tolerance_d_ &&
        odom_v.norm() < 0.1) {
      if (!wait_hover_) {
        pub_hover_p(odom_p, ros::Time::now());
        wait_hover_ = true;
      }
      ROS_WARN("[planner] HOVERING...");
      replanStateMsg_.state = -1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else {
      wait_hover_ = false;
    }
    if (no_need_replan) {
      return;
    }

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
    double replan_t = (replan_stamp - replan_stamp_).toSec();
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      // should replan from the last trajectory
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) =
        iniState;

    // NOTE generate an extra corridor
    Eigen::Vector3d p_start = iniState.col(0);
    bool need_extra_corridor = iniState.col(1).norm() > 1.0;
    Eigen::MatrixXd hPoly;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> line;
    if (need_extra_corridor) {
      Eigen::Vector3d v_norm = iniState.col(1).normalized();
      line.first = p_start;
      double step = 0.1;
      for (double dx = step; dx < 1.0; dx += step) {
        p_start += step * v_norm;
        if (gridmapPtr_->isOccupied(p_start)) {
          p_start -= step * v_norm;
          break;
        }
      }
      line.second = p_start;
      envPtr_->generateOneCorridor(line, 2.0, hPoly);
    }
    // NOTE path searching
    std::vector<Eigen::Vector3d> path;
    bool generate_new_traj_success =
        envPtr_->astar_search(p_start, local_goal, path);
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      if (need_extra_corridor) {
        hPolys.insert(hPolys.begin(), hPoly);
        keyPts.insert(keyPts.begin(), line);
      }
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      // return;
      generate_new_traj_success =
          trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);
      visPtr_->visualize_traj(traj, "traj");
    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      ROS_WARN("[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      // NOTE : if the trajectory is known, watch that direction
      Eigen::Vector3d un_known_p = traj.getPos(
          traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
      Eigen::Vector3d dp = un_known_p - odom_p;
      double yaw = std::atan2(dp.y(), dp.x());
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (!validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  void debug_timer_callback(const ros::TimerEvent& event) {
    inflate_gridmap_pub_.publish(replanStateMsg_.occmap);
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);

    iniState =
        Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3);
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    // std::cout << "target_p: " << target_p.transpose() << std::endl;
    // std::cout << "target_v: " << target_v.transpose() << std::endl;

    // visualize the target and the drone velocity
    visPtr_->visualize_arrow(iniState.col(0), iniState.col(0) + iniState.col(1),
                             "drone_vel");
    visPtr_->visualize_arrow(target_p, target_p + target_v, "target_vel");

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(iniState.col(0), target_p)) {
      visPtr_->visualize_arrow(iniState.col(0), target_p, "ray",
                               visualization::yellow);
    } else {
      visPtr_->visualize_arrow(iniState.col(0), target_p, "ray",
                               visualization::red);
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    if (gridmapPtr_->isOccupied(target_p)) {
      std::cout << "target is invalid!" << std::endl;
      assert(false);
    }
    bool generate_new_traj_success =
        prePtr_->predict(target_p, target_v, target_predcit);

    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(
            observable_p +
            tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    if (generate_new_traj_success) {
      generate_new_traj_success =
          envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    // planning 结果都是在这儿可视化的。
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE generate visible regions
      target_predcit.pop_back();
      way_pts.pop_back();
      envPtr_->generate_visible_regions(target_predcit, way_pts, visible_ps,
                                        thetas);
      visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
      visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas,
                                          "visible_region");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      // TODO change the final state
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
      for (int i = 0; i < (int)way_pts.size(); ++i) {
        rays.emplace_back(target_predcit[i], way_pts[i]);
      }
      visPtr_->visualize_pointcloud(way_pts, "way_pts");
      way_pts.insert(way_pts.begin(), p_start);
      envPtr_->pts2path(way_pts, path);
      visPtr_->visualize_path(path, "corridor_path");
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;
      generate_new_traj_success = trajOptPtr_->generate_traj(
          iniState, finState, target_predcit, visible_ps, thetas, hPolys, traj);
      visPtr_->visualize_traj(traj, "traj");
    }
    if (!generate_new_traj_success) {
      return;
      // assert(false);
    }
    // check
    bool valid = true;
    std::vector<Eigen::Vector3d> check_pts, invalid_pts;
    double t0 = (ros::Time::now() - replan_stamp).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double check_dur = 1.0;
    double delta_t = check_dur < traj.getTotalDuration()
                         ? check_dur
                         : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.1) {
      Eigen::Vector3d p = traj.getPos(t);
      check_pts.push_back(p);
      if (gridmapPtr_->isOccupied(p)) {
        invalid_pts.push_back(p);
      }
    }
    visPtr_->visualize_path(invalid_pts, "invalid_pts");
    visPtr_->visualize_path(check_pts, "check_pts");
    valid = validcheck(traj, replan_stamp);
    if (!valid) {
      std::cout << "invalid!" << std::endl;
    }
  }

  // 时间有效性检查
  bool validcheck(const Trajectory& traj, const ros::Time& t_start,
                  const double& check_dur = 1.0) {
    double t0 = (ros::Time::now() - t_start).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double delta_t = check_dur < traj.getTotalDuration()
                         ? check_dur
                         : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.01) {
      Eigen::Vector3d p = traj.getPos(t);
      if (gridmapPtr_->isOccupied(p)) {
        return false;
      }
    }
    return true;
  }

  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    int plan_hz = 10;
    nh.getParam("plan_hz", plan_hz);
    nh.getParam("tracking_dur", tracking_dur_);
    nh.getParam("tracking_dist", tracking_dist_);
    nh.getParam("tolerance_d", tolerance_d_);
    nh.getParam("debug", debug_);
    nh.getParam("fake", fake_);
    nh.getParam("relative_height", relative_height_);

    gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    envPtr_ = std::make_shared<env::Env>(nh, gridmapPtr_);
    visPtr_ = std::make_shared<visualization::Visualization>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);
    prePtr_ = std::make_shared<prediction::Predict>(nh);

    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("heartbeat", 10);
    traj_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("trajectory", 1);
    replanState_pub_ =
        nh.advertise<quadrotor_msgs::ReplanState>("replanState", 1);

    if (debug_) {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz),
                                   &Nodelet::debug_timer_callback, this);
      // TODO read debug data from files
      wr_msg::readMsg(replanStateMsg_, ros::package::getPath("planning") +
                                           "/../../../debug/replan_state.bin");
      inflate_gridmap_pub_ =
          nh.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 10);
      gridmapPtr_->from_msg(replanStateMsg_.occmap);
      prePtr_->setMap(*gridmapPtr_);
    } else if (fake_) {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz),
                                   &Nodelet::airsim_fake_timer_callback, this);
    } else {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz),
                                   &Nodelet::plan_timer_callback, this);
    }
    gridmap_sub_ = nh.subscribe<quadrotor_msgs::OccMap3d>(
        "gridmap_inflate", 1, &Nodelet::gridmap_callback, this,
        ros::TransportHints().tcpNoDelay());
    // 修改成从airsim中读取位置信息
    // odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10,
    // &Nodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
        "/airsim_node/drone_1/odom_local_enu", 10,
        &Nodelet::airsim_odom_callback, this,
        ros::TransportHints().tcpNoDelay());
    target_sub_ = nh.subscribe<nav_msgs::Odometry>(
        "target", 10, &Nodelet::target_callback, this,
        ros::TransportHints().tcpNoDelay());
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "triger", 10, &Nodelet::triger_callback, this,
        ros::TransportHints().tcpNoDelay());
    land_triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "land_triger", 10, &Nodelet::land_triger_callback, this,
        ros::TransportHints().tcpNoDelay());
    ROS_WARN("Planning node initialized!");
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);