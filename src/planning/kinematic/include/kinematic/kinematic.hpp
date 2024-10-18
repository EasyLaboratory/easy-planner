#include "data_frame.hpp"
#include "math.h"

namespace Kinematic {
class Control {
 public:
  Control() {}
  ~Control() {}

  double calDistance(const Position& ego_Position,
                     const Position& target_Position);
  void normalizeAngle(double* yaw);
  void updateState(State* state);
  void setDisMin(double dis_min) { dis_min_ = dis_min; }
  void setDisMax(double dis_max) { dis_max_ = dis_max; }
  double dismin() { return dis_min_; }
  double dismax() { return dis_max_; }
  void KinematicControl();
  bool Kinematic();
  void updateState();
  void generateEgoTargetState();
  void setStartState(const State& start_state);
  void setTargetState(const State& target_state);
  double flyingHeight() { return flying_height_; }
  State& egoState() { return ego_state_; }
  State& egoTargetState() { return ego_target_state_; }

 private:
  double dt_ = 0.02;
  double dis_min_ = 0.0;
  double dis_max_ = 0.0;
  State ego_state_;
  State target_state_;
  State ego_target_state_;
  double k1_ = 1.0;
  double k2_ = 1.0;
  double vmax_ = 5.0;
  double omegamax_ = 1.5;
  double flying_height_ = 10.0;
  double theta2_ = 70 * M_PI / 180.0;
  double theta1_ = 20 * M_PI / 180.0;
  double Ed_ = 0.0;
};
}  // namespace Kinematic
