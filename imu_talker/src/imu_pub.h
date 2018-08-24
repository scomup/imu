#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>

#include "MPU6050.h"


class Imu_pub
{
  public:
    Imu_pub(int gyro_fs, int accel_fs, bool record);
    void polling();

  private:
    MPU6050 imu_dev_;
    ros::NodeHandle n_;
    ros::Publisher pub_;
    bool record_;
};
