#ifndef MYTOOLS_H
#define MYTOOLS_H

#define NOMINMAX

#include <iostream>
#include <Eigen/Dense>
#include "debug.h"
#include <cmath>
#include <igl/fit_plane.h>

Eigen::Matrix3d getRotationZ(double theta); /* Angle in degrees */
Eigen::Matrix3d getRotationX(double theta); /* Angle in degrees */
Eigen::Matrix3d getRotationY(double theta); /* Angle in degrees */
void calculate_vertex_normal_flann(Eigen::MatrixXd const & V, Eigen::MatrixXd & out_VN);

#endif
