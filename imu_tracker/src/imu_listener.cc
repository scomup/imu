#include "imu_listener.h"

#include <chrono>
#include <fstream>

#include "common.h"

Imu_listener::Imu_listener(ros::NodeHandle* nh): nh_(*nh), quat_(1.0, 0, 0, 0)
{
   imu_sub_ = nh_.subscribe("imu", 1000, &Imu_listener::imuCallback, this);
   last_imu_data_.stamp = 0;
}

void Imu_listener::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_data current_data;
  current_data.stamp = msg->header.stamp.toSec();
  current_data.acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  current_data.gyr = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  if(last_imu_data_.stamp == 0){
    last_imu_data_ = current_data;
    return;
  }
  double dt = current_data.stamp - last_imu_data_.stamp;
  mutex_.lock();
  quatIntegrationStepRK4( quat_, last_imu_data_.gyr, current_data.gyr, dt, quat_ );
  //printf("% 8.3f % 8.3f % 8.3f\n\n",last_imu_data_.gyr(0),last_imu_data_.gyr(1),last_imu_data_.gyr(2));
  mutex_.unlock();
  last_imu_data_ = current_data;
  //std::cout<<std::endl<<"--------------"<<std::endl;
  Eigen::Quaterniond quat(quat_(3),quat_(0),quat_(1),quat_(2));
  
    Eigen::Matrix< double, 3, 1> rpy = quat.toRotationMatrix().eulerAngles(0,1,2);
    const double r = ((double)rpy(0));
    const double p = ((double)rpy(1));
    const double y = ((double)rpy(2));
    //printf("% 8.3f % 8.3f % 8.3f\n",r,p,y);



}

Eigen::Matrix4d Imu_listener::getImuPose(){
  std::unique_lock<std::mutex> lock(mutex_);
  Eigen::Matrix4d m = Eigen::MatrixXd::Identity(4,4);
  Eigen::Quaterniond quat(quat_(0),quat_(1),quat_(2),quat_(3));
  m.block<3,3>(0,0) = quat.toRotationMatrix();
  
  return m;

}


