/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef VIEWER_H
#define VIEWER_H

#include <iostream>
#include <memory>
#include <mutex>

#include "../imu_listener.h"


class Viewer
{
public:
    Viewer(Imu_listener* imu_listner);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void SetFinish();

private:

    double t_;
    double image_width_;
    double image_height_;
    double view_point_x_;
    double view_point_y_;
    double view_point_z_;
    double view_point_f_;
    int grid_update_time_ = 500;
    int path_update_time_ = 50;


    bool finish_;
    Imu_listener* imu_listner_;

    std::mutex mutex_frame_;
    //core::map::HybridGridData hybrid_grid_data_;

};

#endif



