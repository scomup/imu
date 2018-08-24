#if 1
#include "imu_pub.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    Imu_pub imu_pub(MPU6050_GYRO_FS_250, MPU6050_ACCEL_FS_2, false);
}
#else

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <thread>

#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

int end_flag = false;
void set_end_flag()
{
    getchar();
    end_flag = true;
}



int main()
{
    std::ofstream acc_file("acc.txt");
    std::ofstream gyo_file("gyo.txt");

    MPU6050 accelgyro;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    double axs, ays, azs;
    double gxs, gys, gzs;

    double acc_scale, gyo_scale;

    char acc_line[200];
    char gyo_line[200];

    printf("Initializing I2C devices...\n");
    accelgyro.initialize(MPU6050_GYRO_FS_250, MPU6050_ACCEL_FS_2);
    acc_scale = accelgyro.get_acc_scale();
    gyo_scale = accelgyro.get_gyo_scale();

    // verify connection
    printf("Testing device connections...\n");
    printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    auto start = std::chrono::system_clock::now(); 
    
    std::thread t1(set_end_flag);
    while(!end_flag){
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        auto end = std::chrono::system_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()/1000.;

        axs = ax * acc_scale;
        ays = ay * acc_scale;
        azs = az * acc_scale;
        gxs = gx * gyo_scale;
        gys = gy * gyo_scale;
        gzs = gz * gyo_scale;

        printf("% 6.3f % 10.6f % 10.6f % 10.6f % 10.6f % 10.6f % 10.6f\n", elapsed, axs, ays, azs, gxs, gys, gzs);
        sprintf(acc_line, "% 6.3f % 10.6f % 10.6f % 10.6f\n", elapsed, axs, ays, azs);
        sprintf(gyo_line, "% 6.3f % 10.6f % 10.6f % 10.6f\n", elapsed, gxs, gys, gzs);
        acc_file << acc_line;
        gyo_file << gyo_line;
    }
    t1.join();
    acc_file.close();
    gyo_file.close();

}
#endif

