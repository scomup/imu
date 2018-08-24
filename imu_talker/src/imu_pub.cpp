#include "imu_pub.h"

#include <chrono>
#include <fstream>


Imu_pub::Imu_pub(int gyro_fs, int accel_fs, bool record)
:record_(record){
    pub_ = n_.advertise<sensor_msgs::Imu>("imu",1);
    printf("Initializing I2C devices...\n");
    imu_dev_.initialize(gyro_fs, accel_fs);
    printf("Testing device connections...\n");
    if(!imu_dev_.testConnection()){
        printf("MPU6050 connection failed.\n");
        exit(1);
    }
    printf("Device connection successful....\n");
    polling();
}
void Imu_pub::polling(){

    
    std::ofstream acc_file("acc.txt");
    std::ofstream gyo_file("gyo.txt");

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    double axs, ays, azs;
    double gxs, gys, gzs;

    double acc_scale, gyo_scale;

    char acc_line[200];
    char gyo_line[200];
    
    acc_scale = imu_dev_.get_acc_scale();
    gyo_scale = imu_dev_.get_gyo_scale();

    sensor_msgs::Imu imu_data;

    //auto start = std::chrono::system_clock::now(); 
    ros::Time start = ros::Time::now();

    printf("Running....\n");
    while(ros::ok()){
        imu_dev_.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        //auto end = std::chrono::system_clock::now();
        ros::Time current = ros::Time::now();
        //double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()/1000.;
        double elapsed = (current-start).toSec();

        axs = ax * acc_scale;
        ays = ay * acc_scale;
        azs = az * acc_scale;
        gxs = gx * gyo_scale;
        gys = gy * gyo_scale;
        gzs = gz * gyo_scale;

        //printf("% 6.4f % 10.6f % 10.6f % 10.6f % 10.6f % 10.6f % 10.6f\n", elapsed, axs, ays, azs, gxs, gys, gzs);

        if (record_){
            sprintf(acc_line, "% 6.3f % 10.6f % 10.6f % 10.6f\n", elapsed, axs, ays, azs);
            sprintf(gyo_line, "% 6.3f % 10.6f % 10.6f % 10.6f\n", elapsed, gxs, gys, gzs);
            acc_file << acc_line;
            gyo_file << gyo_line;
        }

        ros::Time current_time = ros::Time::now();
        imu_data.header.stamp = current_time;
        imu_data.linear_acceleration.x = axs;
        imu_data.linear_acceleration.y = ays;
        imu_data.linear_acceleration.z = azs;
        imu_data.angular_velocity.x = gxs;
        imu_data.angular_velocity.y = gys;
        imu_data.angular_velocity.z = gzs;
        pub_.publish(imu_data);
    }
    printf("Finished....\n");
    acc_file.close();
    gyo_file.close();
}


