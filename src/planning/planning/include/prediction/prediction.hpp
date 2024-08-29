#pragma once

#include <mapping/mapping.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <queue>

namespace prediction {

struct Node {
  Eigen::Vector3d p, v, a;
  double t;
  double score;
  double h;
  Node* parent = nullptr;
};
typedef Node* NodePtr;
class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) {
    return lhs->score + lhs->h > rhs->score + rhs->h;
  }
};
struct Predict {
 private:
  static constexpr int MAX_MEMORY = 1 << 22;
  // searching

  double dt;
  double pre_dur;
  double rho_a;
  double car_z, vmax;
  mapping::OccGridMap map;
  NodePtr data[MAX_MEMORY];
  int stack_top;

  // 判断位置和速度是否合理
  inline bool isValid(const Eigen::Vector3d& p,
                      const Eigen::Vector3d& v) const {
    return (v.norm() < vmax) && (!map.isOccupied(p));
  }

 public:
  inline Predict(ros::NodeHandle& nh) {
    // pre_dur = 3.0
    // 颗粒度是0.2s
    nh.getParam("tracking_dur", pre_dur);
    nh.getParam("tracking_dt", dt);
    // rho_a = 1.0
    // 加速度的评分评分权重系数
    nh.getParam("prediction/rho_a", rho_a);
    nh.getParam("prediction/vmax", vmax);
    for (int i = 0; i < MAX_MEMORY; ++i) {
      data[i] = new Node;
    }
  }
  inline void setMap(const mapping::OccGridMap& _map) {
    map = _map;
    // map.inflate_last();
  }

  // 实际用到的输出预测轨迹的代码。
  inline bool predict(const Eigen::Vector3d& target_p,
                      const Eigen::Vector3d& target_v,
                      std::vector<Eigen::Vector3d>& target_predcit,
                      const double& max_time = 0.1) {
    // 根据加速度最小打分
    auto score = [&](const NodePtr& ptr) -> double {
      return rho_a * ptr->a.norm();
    };
    Eigen::Vector3d end_p = target_p + target_v * pre_dur;
    // 定义一个lambda表达式，用于计算节点的启发式值，基于节点位置和预测结束位置的距离
    auto calH = [&](const NodePtr& ptr) -> double {
      return 0.001 * (ptr->p - end_p).norm();
    };
    ros::Time t_start = ros::Time::now();
    // 定义好的比较方式，用了一个优先队列来完成这个事情。
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    Eigen::Vector3d input(0, 0, 0);

    stack_top = 0;
    NodePtr curPtr = data[stack_top++];
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;
    double dt2_2 = dt * dt / 2;
    // 做了一个采样，根据加速度上下限制-3到+3采样，颗粒度为3。
    while (curPtr->t < pre_dur) {
      ROS_INFO("curPtr->t = %f", curPtr->t);
      for (input.x() = -3; input.x() <= 3; input.x() += 3)
        for (input.y() = -3; input.y() <= 3; input.y() += 3) {
          // 位置和速度作匀加速运动的预测。
          Eigen::Vector3d p = curPtr->p + curPtr->v * dt + input * dt2_2;
          Eigen::Vector3d v = curPtr->v + input * dt;
          bool tmp = isValid(p, v);
          ROS_INFO("input (x, y) = (%f, %f), isValid = %s", input.x(), input.y(), tmp ? "true" : "false");
          if (!isValid(p, v)) {
            continue;
          }
          // 检查是否超出内存限制
          if (stack_top == MAX_MEMORY) {
            std::cout << "[prediction] out of memory!" << std::endl;
            return false;
          }
          // 检查是否超过最大允许时间
          double t_cost = (ros::Time::now() - t_start).toSec();
          if (t_cost > max_time) {
            std::cout << "[prediction] too slow!" << std::endl;
            return false;
          }
          NodePtr ptr = data[stack_top++];
          ptr->p = p;
          ptr->v = v;
          ptr->a = input;
          ptr->parent = curPtr;
          ptr->t = curPtr->t + dt;
          // 对分数做累加
          ptr->score = curPtr->score + score(ptr);
          ptr->h = calH(ptr);
          open_set.push(ptr);
          // std::cout << "open set push: " << state.transpose() << std::endl;
        }
      if (open_set.empty()) {
        // open_set.push(curPtr);
        std::cout << "[prediction] no way!" << std::endl;
        return false;
      }
      curPtr = open_set.top();
      open_set.pop();
    }
    target_predcit.clear();
    // 逆向回溯路径，并将路径节点位置加入预测结果向量
    while (curPtr != nullptr) {
      target_predcit.push_back(curPtr->p);
      curPtr = curPtr->parent;
    }
    std::reverse(target_predcit.begin(), target_predcit.end());
    return true;
  }
};

}  // namespace prediction
