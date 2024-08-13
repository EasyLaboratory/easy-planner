#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
// #include <trajectory_msgs/TrajectoryPoint.h>
#include <visualization_msgs/Marker.h>

#include <traj_opt/poly_traj_utils.hpp>

ros::Publisher pos_cmd_pub_;
ros::Publisher airsim_pos_cmd_pub_;
ros::Time heartbeat_time_;
bool receive_traj_ = false;
bool flight_start_ = false;
quadrotor_msgs::PolyTraj trajMsg_, trajMsg_last_;
Eigen::Vector3d last_p_;
double last_yaw_ = 0;

inline void normalizeYaw(double *yaw) {
  *yaw = fmod(*yaw, 2 * M_PI);
  if (*yaw < 0) {
    *yaw += 2 * M_PI;
  }
}

void publish_cmd(int traj_id, const Eigen::Vector3d &p,
                 const Eigen::Vector3d &v, const Eigen::Vector3d &a, double y,
                 double yd) {
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag =
      quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub_.publish(cmd);

  // 发送消息给airsim
  mavros_msgs::PositionTarget point;
  point.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  point.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

  point.position.x = p(0);
  point.position.y = p(1);
  point.position.z = p(2);

  point.velocity.x = v(0);
  point.velocity.y = v(1);
  point.velocity.z = v(2);  // Assume constant altitude

  normalizeYaw(&y);  // Normalize yaw
  point.yaw = y - M_PI / 2.0;
  ROS_INFO("TO airsim point (x,y,z) = (%f,%f,%f)", point.position.x,
           point.position.y, point.position.z);
  ROS_INFO("TO airsim yaw = (%f)", point.yaw);
  ROS_INFO("TO airsim point (vx,vy,vz) = (%f,%f,%f)", point.velocity.x,
           point.velocity.y, point.velocity.z);
  airsim_pos_cmd_pub_.publish(point);
  last_p_ = p;
}

bool exe_traj(const quadrotor_msgs::PolyTraj &trajMsg) {
  double t = (ros::Time::now() - trajMsg.start_time).toSec();
  if (t > 0) {
    if (trajMsg.hover) {
      if (trajMsg.hover_p.size() != 3) {
        ROS_ERROR("[traj_server] hover_p is not 3d!");
      }
      Eigen::Vector3d p, v0;
      p.x() = trajMsg.hover_p[0];
      p.y() = trajMsg.hover_p[1];
      p.z() = trajMsg.hover_p[2];
      v0.setZero();
      publish_cmd(trajMsg.traj_id, p, v0, v0, last_yaw_, 0);  // TODO yaw
      return true;
    }
    if (trajMsg.order != 5) {
      ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
      return false;
    }
    if (trajMsg.duration.size() * (trajMsg.order + 1) !=
        trajMsg.coef_x.size()) {
      ROS_ERROR("[traj_server] WRONG trajectory parameters!");
      return false;
    }
    // 轨迹的时长
    int piece_nums = trajMsg.duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
      int i6 = i * 6;
      cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1],
          trajMsg.coef_x[i6 + 2], trajMsg.coef_x[i6 + 3],
          trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
      cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1],
          trajMsg.coef_y[i6 + 2], trajMsg.coef_y[i6 + 3],
          trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
      cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1],
          trajMsg.coef_z[i6 + 2], trajMsg.coef_z[i6 + 3],
          trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];

      dura[i] = trajMsg.duration[i];
    }
    Trajectory traj(dura, cMats);
    if (t > traj.getTotalDuration()) {
      ROS_ERROR("[traj_server] trajectory too short left!");
      return false;
    }
    Eigen::Vector3d p, v, a;
    p = traj.getPos(t);
    v = traj.getVel(t);
    a = traj.getAcc(t);
    // NOTE yaw
    double yaw = trajMsg.yaw;
    double d_yaw = yaw - last_yaw_;
    d_yaw = d_yaw >= M_PI ? d_yaw - 2 * M_PI : d_yaw;
    d_yaw = d_yaw <= -M_PI ? d_yaw + 2 * M_PI : d_yaw;
    double d_yaw_abs = fabs(d_yaw);
    if (d_yaw_abs >= 0.02) {
      yaw = last_yaw_ + d_yaw / d_yaw_abs * 0.02;
    }
    // 输出的量有位置，速度，加速度，航向角，航向角速度。
    publish_cmd(trajMsg.traj_id, p, v, a, yaw, 0);  // TODO yaw
    last_yaw_ = yaw;
    return true;
  }
  return false;
}

void heartbeatCallback(const std_msgs::EmptyConstPtr &msg) {
  heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(const quadrotor_msgs::PolyTrajConstPtr &msgPtr) {
  trajMsg_ = *msgPtr;
  if (!receive_traj_) {
    trajMsg_last_ = trajMsg_;
    receive_traj_ = true;
  }
}

void cmdCallback(const ros::TimerEvent &e) {
  if (!receive_traj_) {
    ROS_WARN("we dont receive traj!!");
    return;
  }
  ros::Time time_now = ros::Time::now();
  if ((time_now - heartbeat_time_).toSec() > 0.5) {
    ROS_ERROR_ONCE(
        "[traj_server] Lost heartbeat from the planner, is he dead?");
    publish_cmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero(), 0, 0);  // TODO yaw
    return;
  }
  if (exe_traj(trajMsg_)) {
    trajMsg_last_ = trajMsg_;
    return;
  } else if (exe_traj(trajMsg_last_)) {
    return;
  }
}

// 这个函数的功能是给rviz中的目标或者ego发送轨迹。
int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");
  // 这个句柄是用来发送给airsim的。
  ros::NodeHandle n;

  ros::Subscriber poly_traj_sub =
      nh.subscribe("trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub =
      nh.subscribe("heartbeat", 10, heartbeatCallback);

  // 实际的轨迹点是从这儿发出去的。
  pos_cmd_pub_ =
      nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);
  // 这个是我的加的给airsim的
  airsim_pos_cmd_pub_ =
      n.advertise<mavros_msgs::PositionTarget>("/command/trajectory", 50);

  // 定时去收位置请求。
  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.05), cmdCallback);

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}