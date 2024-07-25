#include <ros/ros.h>
#include <trajectory_msgs/TrajectoryPoint.h>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include <math.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// 生成圆形轨迹并包含速度和航向角信息

std::vector<trajectory_msgs::TrajectoryPoint> generate_circle_trajectory(double radius, double altitude, int num_points, double speed) {
    std::vector<trajectory_msgs::TrajectoryPoint> trajectory;

    for (int i = 0; i < num_points; ++i) {
        double t = 2 * M_PI * i / num_points;
        double x = radius * cos(t);
        double y = radius * sin(t);

        trajectory_msgs::TrajectoryPoint point;
        point.pose.position.x = x;
        point.pose.position.y = y;
        point.pose.position.z = altitude;

        // Calculate velocity
        double vx = -radius * sin(t) * speed;
        double vy = radius * cos(t) * speed;

        point.velocity.linear.x = vx;
        point.velocity.linear.y = vy;
        point.velocity.linear.z = 0; // Assume constant altitude

        // Calculate yaw angle
        double yaw = atan2(vy, vx);

        // Convert yaw angle to quaternion
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        point.pose.orientation.x = q.x();
        point.pose.orientation.y = q.y();
        point.pose.orientation.z = q.z();
        point.pose.orientation.w = q.w();

        trajectory.push_back(point);
    }

    return trajectory;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle n;

    ros::Publisher traj_pub = n.advertise<trajectory_msgs::TrajectoryPoint>("command/trajectory", 10);

    double radius = 5.0;  // 圆形轨迹的半径
    double altitude = -5.0; // 飞行的高度
    int num_points = 100;  // 设置轨迹点的数量
    double speed = 1;   // 飞行速度

    auto trajectory = generate_circle_trajectory(radius, altitude, num_points, speed);

    // Calculate distance between consecutive points
    double distance = 2 * M_PI * radius / num_points;

    // Calculate publish rate based on speed and distance
    double publish_rate = speed / distance;

    ros::Rate rate(20);
    int point_index = 0;

    while (ros::ok()) {
        traj_pub.publish(trajectory[point_index]);
        point_index = (point_index + 1) % num_points;  // 循环发布轨迹点
        rate.sleep();
    }

    return 0;
}
