#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <target_ekf/target_ekf.hpp>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                        nav_msgs::Odometry>
    YoloOdomSyncPolicy;
typedef message_filters::Synchronizer<YoloOdomSyncPolicy> YoloOdomSynchronizer;
ros::Publisher target_odom_pub_, yolo_odom_pub_;
Eigen::Matrix3d cam2body_R_;
Eigen::Vector3d cam2body_p_;
double fx_, fy_, cx_, cy_, width_, height_;
ros::Time last_update_stamp_;
double pitch_thr_ = 30;
bool check_fov_ = false;

std::shared_ptr<Ekf> ekfPtr_;

// 定时启动的回调函数
void predict_state_callback(const ros::TimerEvent& event) {
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  // std::cout << "predict_state_callback----------" << std::endl;
  if (update_dt < 2.0) {
    ekfPtr_->predict();
  } else {
    ROS_WARN("[ekf] too long time target position no update!");
    return;
  }
  // publish target odom
  nav_msgs::Odometry target_odom;
  target_odom.header.stamp = ros::Time::now();
  target_odom.header.frame_id = "world";
  target_odom.pose.pose.position.x = ekfPtr_->pos().x();
  target_odom.pose.pose.position.y = ekfPtr_->pos().y();
  target_odom.pose.pose.position.z = ekfPtr_->pos().z();
  target_odom.twist.twist.linear.x = ekfPtr_->vel().x();
  target_odom.twist.twist.linear.y = ekfPtr_->vel().y();
  target_odom.twist.twist.linear.z = ekfPtr_->vel().z();
  Eigen::Vector3d rpy = ekfPtr_->rpy();
  Eigen::Quaterniond q = euler2quaternion(rpy);
  target_odom.pose.pose.orientation.w = q.w();
  target_odom.pose.pose.orientation.x = q.x();
  target_odom.pose.pose.orientation.y = q.y();
  target_odom.pose.pose.orientation.z = q.z();
  target_odom_pub_.publish(target_odom);
}

// 同步处理yolo和odom的回调函数,处理目标物体的位置和定位信息
void update_state_callback(const nav_msgs::OdometryConstPtr& target_msg,
                           const nav_msgs::OdometryConstPtr& odom_msg) {
  // std::cout << "yolo stamp: " << bboxes_msg->header.stamp << std::endl;
  // std::cout << "odom stamp: " << odom_msg->header.stamp << std::endl;
  // ***********************这里的东西没有被用到 ***********************
  Eigen::Vector3d odom_p, p;
  Eigen::Quaterniond odom_q, q;
  odom_p(0) = odom_msg->pose.pose.position.x;
  odom_p(1) = odom_msg->pose.pose.position.y;
  odom_p(2) = odom_msg->pose.pose.position.z;
  odom_q.w() = odom_msg->pose.pose.orientation.w;
  odom_q.x() = odom_msg->pose.pose.orientation.x;
  odom_q.y() = odom_msg->pose.pose.orientation.y;
  odom_q.z() = odom_msg->pose.pose.orientation.z;

  Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;
  Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);
  // ***********************这里的东西没有被用到***********************

  p.x() = target_msg->pose.pose.position.x;
  p.y() = target_msg->pose.pose.position.y;
  p.z() = target_msg->pose.pose.position.z;
  q.w() = target_msg->pose.pose.orientation.w;
  q.x() = target_msg->pose.pose.orientation.x;
  q.y() = target_msg->pose.pose.orientation.y;
  q.z() = target_msg->pose.pose.orientation.z;
  ROS_INFO("through kalam target postion (x,y,z) = (%f,%f,%f)", p.x(), p.y(),
           p.z());

  Eigen::Vector3d rpy = quaternion2euler(q);

  // NOTE check whether it's in FOV
  // check_fov_ = false 这个函数没有进来
  if (check_fov_) {
    Eigen::Vector3d p_in_body = cam_q.inverse() * (p - cam_p);
    if (p_in_body.z() < 0.1 || p_in_body.z() > 5.0) {
      return;
    }
    double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
    if (x < 0 || x > height_) {
      return;
    }
    double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
    if (y < 0 || y > width_) {
      return;
    }
  }

  // update target odom，得到目标物的信息
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt > 5.0) {
    ekfPtr_->reset(p, rpy);
    ROS_WARN("[ekf] reset!");
  } else if (ekfPtr_->update(p, rpy)) {
    // 这就是代表目标位置实际更新了！！！
    ROS_WARN("[ekf] update!");
  } else {
    ROS_ERROR("[ekf] update invalid!");
    return;
  }
  last_update_stamp_ = ros::Time::now();
}

void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg) {}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_ekf");
  ros::NodeHandle nh("~");
  last_update_stamp_ = ros::Time::now() - ros::Duration(10.0);

  // 关于相机的参数
  std::vector<double> tmp;
  if (nh.param<std::vector<double>>("cam2body_R", tmp, std::vector<double>())) {
    cam2body_R_ =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            tmp.data(), 3, 3);
  }
  if (nh.param<std::vector<double>>("cam2body_p", tmp, std::vector<double>())) {
    cam2body_p_ =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            tmp.data(), 3, 1);
  }
  nh.getParam("cam_fx", fx_);
  nh.getParam("cam_fy", fy_);
  nh.getParam("cam_cx", cx_);
  nh.getParam("cam_cy", cy_);
  nh.getParam("cam_width", width_);
  nh.getParam("cam_height", height_);
  nh.getParam("pitch_thr", pitch_thr_);
  nh.getParam("check_fov", check_fov_);

  message_filters::Subscriber<nav_msgs::Odometry> yolo_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  std::shared_ptr<YoloOdomSynchronizer> yolo_odom_sync_Ptr_;
  ros::Timer ekf_predict_timer_;
  ros::Subscriber single_odom_sub = nh.subscribe(
      "odom", 100, &odom_callback, ros::TransportHints().tcpNoDelay());
  // 这是卡尔曼滤波完之后发布出去的目标点的位姿信息,给planning的
  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("target_odom", 1);
  // 这个没有被用到
  yolo_odom_pub_ = nh.advertise<nav_msgs::Odometry>("yolo_odom", 1);

  int ekf_rate = 50;
  nh.getParam("ekf_rate", ekf_rate);
  ekfPtr_ = std::make_shared<Ekf>(1.0 / ekf_rate);

  // 订阅 YOLO 和 Odom 的里程计消息，并在这些消息同步到达时调用
  // update_state_callback 函数。
  yolo_sub_.subscribe(nh, "yolo", 1, ros::TransportHints().tcpNoDelay());
  odom_sub_.subscribe(nh, "odom", 100, ros::TransportHints().tcpNoDelay());
  yolo_odom_sync_Ptr_ = std::make_shared<YoloOdomSynchronizer>(
      YoloOdomSyncPolicy(200), yolo_sub_, odom_sub_);
  yolo_odom_sync_Ptr_->registerCallback(
      boost::bind(&update_state_callback, _1, _2));

  // 每隔这个时间调用一次预测轨迹函数ros::Duration(1.0 / ekf_rate) =
  // 0.05s，频率就是20Hz。
  ekf_predict_timer_ =
      nh.createTimer(ros::Duration(1.0 / ekf_rate), &predict_state_callback);

  ros::spin();
  return 0;
}
