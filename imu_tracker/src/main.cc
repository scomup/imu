#include <thread>
#include "ros/ros.h"
#include "viewer/viewer.h"
#include "imu_listener.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_tracker");
    ros::NodeHandle nh;
    auto imu_listner = new Imu_listener(&nh);
    
    auto viewer = new Viewer(imu_listner);
    auto viewer_thread = std::thread(&Viewer::Run, viewer);
    ros::spin();
    std::cout<<"endl"<<std::endl;
}