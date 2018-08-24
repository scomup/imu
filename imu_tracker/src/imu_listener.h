#ifndef IMU_LISTENER_H
#define IMU_LISTENER_H

#include <mutex>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include "Eigen/Core"
#include "Eigen/Geometry"

struct imu_data
{
  double stamp;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyr;
};

class Imu_listener
{
  public:
    Imu_listener(ros::NodeHandle *nh);
    Eigen::Matrix4d getImuPose();

  private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    imu_data last_imu_data_;
    Eigen::Vector4d quat_;
    std::mutex mutex_;
};

#endif
