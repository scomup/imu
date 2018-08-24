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

#include "viewer.h"
#include <pangolin/pangolin.h>
#include "myhandler.h"

#include "draw_object_helper.h"
#include "frustum_culling.h"

#include "Eigen/Core"
#include "Eigen/Geometry"


#include <iostream>
#include <chrono>
#include <thread>

Viewer::Viewer(Imu_listener* imu_listner):imu_listner_(imu_listner)
{
    t_ = 1e3/30;
    image_width_ = 640;
    image_height_ = 480;
    view_point_x_ = -0.1;
    view_point_y_ = -2;
    view_point_z_ = 1;
    view_point_f_ = 2000;
}

void Viewer::Run()
{
    finish_ = false;

    pangolin::CreateWindowAndBind("sample_carto_3d viewer",1024,768);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuShowPath("menu.show path",true,true);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,view_point_f_,view_point_f_,512,389,0.1,1000),
                pangolin::ModelViewLookAt(view_point_x_,view_point_y_,view_point_z_, 0,0,0, 0.0 ,0.0,1.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::MyHandler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    bool bFollow = true;

    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    auto last = std::chrono::system_clock::now();
    double roll=0,pitch=0,yaw=0;
    //Eigen::Quaternionf quat = RollPitchYaw(roll,pitch, yaw);
    Eigen::Affine3f rotation = RollPitchYaw(0, 0, 0);

    
    
   //auto lastTime = std::chrono::system_clock::now();


    while( !pangolin::ShouldQuit() && !finish_)
    {
        auto current = std::chrono::system_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current-last).count()/1000.;
        if(elapsed >= 0.01){
            last = current;
            //quat = RollPitchYaw(roll,pitch, yaw);
            //yaw += 0.1;
            pitch += 0.1;
        }



        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        d_cam.Activate(s_cam);
        /*
        Eigen::Affine3f trans(Eigen::Translation3f(Eigen::Vector3f(0,0,0)));
        Eigen::Affine3f rotation = RollPitchYaw(roll,pitch, yaw);
        Eigen::Matrix4f m = (trans * rotation).matrix(); // Option 1
        //m *= r.matrix();
        */

        pangolin::glDrawAxis(1);
        DrawGrid(2,0.1);
        Eigen::Matrix4d m = imu_listner_->getImuPose();
        //std::cout<<m<<std::endl;
        DrawIMU(m.data());
        //if(menuShowPath)
        //    DrawPath();

        pangolin::FinishFrame();
    }
}



void Viewer::SetFinish()
{
    finish_ = true;
}



