#include "kinematic/kinematic.hpp"

#include <ros/ros.h>

#include <iostream>
// using namespace std;

namespace Kinematic {
double Control::calDistance(const Position& ego_Position,
                            const Position& target_Position) {
  return std::hypot(ego_Position.x - target_Position.x,
                    ego_Position.y - target_Position.y);
}

void Control::normalizeAngle(double* yaw) {
  // 让yaw值在[-pi, pi]区间内
  while (*yaw < -M_PI) {
    *yaw += 2 * M_PI;
  }
  while (*yaw > M_PI) {
    *yaw -= 2 * M_PI;
  }
}

void Control::setStartState(const State& start_state) {
  ego_state_ = start_state;
  // Position pos;
  // pos.x = start_state.pos().x;
  // pos.y = start_state.pos().y;
  // pos.z = start_state.pos().z;
  // ego_state_.setPos(pos);
}

void Control::setTargetState(const State& target_state) {
  target_state_ = target_state;
  // Position pos;
  // pos.x = target_state.pos().x;
  // pos.y = target_state.pos().y;
  // pos.z = flying_height_;
  // target_state_.setPos(pos);
  // std::cout << "target_state_.pos().x = " << target_state_.pos().x
  //           << ", target_state_.pos().y =" << target_state_.pos().y
  //           << std::endl;
}

void Control::updateState(State* state) {
  state->setPos(Position(state->pos().x + state->vel().vx * dt_,
                         state->pos().y + state->vel().vy * dt_,
                         state->pos().z));
  state->setYaw(state->yaw() + state->omega() * dt_);
}

void Control::updateState() {
  ego_state_.setPos(Position(ego_state_.pos().x + ego_state_.vel().vx * dt_,
                             ego_state_.pos().y + ego_state_.vel().vy * dt_,
                             ego_state_.pos().y));
  ego_state_.setYaw(ego_state_.yaw() + ego_state_.omega() * dt_);
}

void Control::KinematicControl() {
  // 这些是为了让无人机在目标后面的一定范围内，根据高度确定跟踪范围
  double distance = calDistance(ego_state_.pos(), target_state_.pos());
  double Ed_max = ego_state_.pos().z * std::tan(theta2_);
  double Ed_min = ego_state_.pos().z * std::tan(theta1_);
  if (distance < Ed_min) {
    Ed_ = Ed_min;
  } else if (distance > Ed_max) {
    Ed_ = Ed_max;
  } else {
    Ed_ = 0.0;
  }
  double epsilon_distance = distance - Ed_;
  std::cout << "distance = " << distance << std::endl;
  std::cout << "epsilon_distance = " << epsilon_distance << std::endl;
  // 当前位置和目标物体连线的角度
  double tao_d = std::atan2(target_state_.pos().y - ego_state_.pos().y,
                            target_state_.pos().x - ego_state_.pos().x);
  // 目标物体的航向角
  double tao_t = std::atan2(target_state_.vel().vy, target_state_.vel().vx);
  double vt = std::hypot(target_state_.vel().vx, target_state_.vel().vy);
  // 和目标位置计算得到的当前角度值
  double epsilon_tao = tao_d;
  double vx = k1_ * epsilon_distance * std::cos(epsilon_tao) +
              vt * std::cos(tao_t - tao_d) * std::cos(epsilon_tao);
  if (vx > vmax_) {
    vx = vmax_;
  }
  if (vx < vmax_ * (-1.0)) {
    vx = -1 * vmax_;
  }
  double vy = k1_ * epsilon_distance * std::sin(epsilon_tao) +
              vt * std::cos(tao_t - tao_d) * std::sin(epsilon_tao);
  if (vy > vmax_) {
    vy = vmax_;
  }
  if (vx < vmax_ * (-1.0)) {
    vy = -1 * vmax_;
  }
  Vel vel(vx, vy, 0.0);
  ego_state_.setVel(vel);
  // normalizeAngle(&target_yaw);
  // ego_state_.setYaw(target_yaw);
  double omega =
      k2_ * epsilon_tao + vt / epsilon_distance * std::sin(tao_t - tao_d);
  if (omega > omegamax_) {
    omega = omegamax_;
  }
  if (omega < omegamax_ * (-1.0)) {
    omega = -1 * omegamax_;
  }
  updateState();
}

void Control::generateEgoTargetState() {
  Position pos;
  pos.x = ego_state_.pos().x + ego_state_.vel().vx * dt_;
  pos.y = ego_state_.pos().y + ego_state_.vel().vy * dt_;
  pos.z = flying_height_;
  ego_target_state_.setPos(pos);
  double yaw = ego_state_.yaw() + ego_state_.omega() * dt_;
  ego_target_state_.setYaw(yaw);
}

bool Control::Kinematic() {
  // 这些是为了让无人机在目标后面的一定范围内，根据高度确定跟踪范围
  double distance = calDistance(ego_state_.pos(), target_state_.pos());
  double Ed_max = ego_state_.pos().z * std::tan(theta2_);
  double Ed_min = ego_state_.pos().z * std::tan(theta1_);
  if (distance < Ed_min) {
    Ed_ = Ed_min;
  } else if (distance > Ed_max) {
    Ed_ = Ed_max;
  } else {
    Ed_ = 0.0;
  }
  ROS_INFO("Ed_max = %f", Ed_max);
  ROS_INFO("Ed_min = %f", Ed_min);
  double epsilon_distance = distance - Ed_;
  ROS_INFO("distance = %f", distance);
  ROS_INFO("epsilon_distance = %f", epsilon_distance);
  // 当前位置和目标物体连线的角度
  double tao_d = std::atan2(target_state_.pos().y - ego_state_.pos().y,
                            target_state_.pos().x - ego_state_.pos().x);
  // 目标物体的航向角
  double tao_t = std::atan2(target_state_.vel().vy, target_state_.vel().vx);
  double vt = std::hypot(target_state_.vel().vx, target_state_.vel().vy);
  // 和目标位置计算得到的当前角度值
  double epsilon_tao = tao_d;
  double vx = k1_ * epsilon_distance * std::cos(epsilon_tao) +
              vt * std::cos(tao_t - tao_d) * std::cos(epsilon_tao);
  if (vx > vmax_) {
    vx = vmax_;
  }
  if (vx < vmax_ * (-1.0)) {
    vx = -1 * vmax_;
  }
  double vy = k1_ * epsilon_distance * std::sin(epsilon_tao) +
              vt * std::cos(tao_t - tao_d) * std::sin(epsilon_tao);
  if (vy > vmax_) {
    vy = vmax_;
  }
  if (vx < vmax_ * (-1.0)) {
    vy = -1 * vmax_;
  }
  Vel vel(vx, vy, 0.0);
  ego_target_state_.setVel(vel);
  double omega =
      k2_ * epsilon_tao + vt / epsilon_distance * std::sin(tao_t - tao_d);
  if (omega > omegamax_) {
    omega = omegamax_;
  }
  if (omega < omegamax_ * (-1.0)) {
    omega = -1 * omegamax_;
  }
  ego_target_state_.setOmega(omega);
  generateEgoTargetState();
  return true;
}

}  // namespace Kinematic
