#include "kinematic/kinematic.hpp"

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
}

void Control::setTargetState(const State& target_state) {
  target_state_ = target_state;
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
  // std::cout << "ego_state_.vel().vx = " << ego_state_.vel().vx
  //           << ", ego_state_.vel().vy =" << ego_state_.vel().vy << std::endl;
  // std::cout << "ego_state_.pos().x = " << ego_state_.pos().x
  //           << ", ego_state_.pos().y =" << ego_state_.pos().y << std::endl;
  ego_state_.setYaw(ego_state_.yaw() + ego_state_.omega() * dt_);
}

void Control::KinematicControl() {
  double epsilon_distance = calDistance(ego_state_.pos(), target_state_.pos());
  std::cout << "epsilon_distance = " << epsilon_distance << std::endl;

  double epsilon_yaw = std::atan2(-ego_state_.pos().y + target_state_.pos().y,
                                  -ego_state_.pos().x + target_state_.pos().x);
  std::cout << "epsilon_yaw = " << epsilon_yaw << std::endl;

  double target_yaw = std::atan2(ego_state_.vel().vx, target_state_.vel().vy);
  std::cout << "target_yaw = " << target_yaw << std::endl;

  double vx = k1_ * epsilon_distance * std::cos(epsilon_yaw) +
              ego_state_.vel().vx * std::cos(target_yaw - epsilon_yaw);
  std::cout << "vx = " << vx << std::endl;

  double vy = k1_ * epsilon_distance * std::sin(epsilon_yaw) +
              ego_state_.vel().vy * std::cos(target_yaw - epsilon_yaw);
  std::cout << "vy = " << vy << std::endl;
  Vel vel(vx, vy, 0.0);
  ego_state_.setVel(vel);
  double omega = k2_ * epsilon_yaw + std::sqrt(vx * vx + vy * vy) /
                                         epsilon_distance *
                                         std::sin(target_yaw - epsilon_yaw);
  std::cout << "omega = " << omega << std::endl;
  updateState();
}

bool Control::Kinematic() {
  double epsilon_distance = calDistance(ego_state_.pos(), target_state_.pos());

  double epsilon_yaw = std::atan2(-ego_state_.pos().y + target_state_.pos().y,
                                  -ego_state_.pos().x + target_state_.pos().x);

  double target_yaw = std::atan2(ego_state_.vel().vx, target_state_.vel().vy);

  double vx = k1_ * epsilon_distance * std::cos(epsilon_yaw) +
              ego_state_.vel().vx * std::cos(target_yaw - epsilon_yaw);

  double vy = k1_ * epsilon_distance * std::sin(epsilon_yaw) +
              ego_state_.vel().vy * std::cos(target_yaw - epsilon_yaw);

  Vel vel(vx, vy, 0.0);
  ego_state_.setVel(vel);
  double omega = k2_ * epsilon_yaw + std::sqrt(vx * vx + vy * vy) /
                                         epsilon_distance *
                                         std::sin(target_yaw - epsilon_yaw);
  // std::cout << "omega = " << omega << std::endl;
  // updateState();
  return true;
}

}  // namespace Kinematic
