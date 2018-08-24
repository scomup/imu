#ifndef SAMPLE_CARTO_TOP_VIEWER_DRAW_OBJECT_HELPER_H_
#define SAMPLE_CARTO_TOP_VIEWER_DRAW_OBJECT_HELPER_H_


#include <pangolin/pangolin.h>
#include "Eigen/Core"
#include "Eigen/Geometry"

/*
Eigen::Quaternionf RollPitchYaw(const double roll, const double pitch,
                                const double yaw) {
  const Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}*/

Eigen::Affine3f RollPitchYaw(const double roll, const double pitch,const double yaw) {
  const Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(roll, Eigen::Vector3f(1, 0, 0)));
  const Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(pitch, Eigen::Vector3f(0, 1, 0)));
  const Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(yaw, Eigen::Vector3f(0, 0, 1)));
  return rz * ry * rx;
}


inline void DrawIMU(const double* twc)
{
    glPushMatrix();
    glMultMatrixd(twc);
    GLdouble r = 0.1;
    GLdouble l[3] = {- r/ 2, -r / 2, -r / 2};
    GLdouble h[3] = {+ r/ 2, +r / 2, +r / 2};
    //Front
    glBegin(GL_QUADS);
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(l[0], l[1], h[2]);
    glVertex3f(l[0], l[1], l[2]);
    //Back
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(l[0], h[1], l[2]);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(h[0], h[1], h[2]);
    glVertex3f(h[0], h[1], l[2]);
    //Top
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(h[0], h[1], h[2]);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(l[0], l[1], h[2]);
    //Bottom
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(h[0], h[1], l[2]);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(l[0], l[1], l[2]);
    glVertex3f(l[0], h[1], l[2]);
    //Left
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(l[0], h[1], h[2]);
    glVertex3f(l[0], h[1], l[2]);
    glVertex3f(l[0], l[1], l[2]);
    glVertex3f(l[0], l[1], h[2]);
    //Right
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(h[0], l[1], h[2]);
    glVertex3f(h[0], l[1], l[2]);
    glVertex3f(h[0], h[1], l[2]);
    glVertex3f(h[0], h[1], h[2]);
    glEnd();
    glPopMatrix();
}


GLint DrawGrid(GLdouble w, GLdouble s)
{
    glLineWidth(1);
    glColor4f(0.0f,0.0f,0.0f,0.3f);
    for (double i = -w / 2; i <= w / 2; i+=s)
    {
        pangolin::glDrawLine(i, -w / 2, 0, i, w / 2, 0);
        pangolin::glDrawLine(-w / 2, i, 0, w / 2, i, 0);
    };
    return (0);
}

#endif //SAMPLE_CARTO_TOP_VIEWER_DRAW_OBJECT_HELPER_H_
