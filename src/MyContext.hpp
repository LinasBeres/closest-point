#ifndef MYCONTEXT_H
#define MYCONTEXT_H

#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/writeOFF.h>
#include <igl/file_exists.h>
#include <imgui/imgui.h>
#include <iostream>
#include <random>

#include "debug.h"
#include "mytools.h"
#include "ICP.hpp"

class MyContext
{
  public:
    MyContext();
    ~MyContext();

    Eigen::MatrixXd m_V, m_V2, m_V3, test, m_V4, m_V5, m_V6, m_V7, m_V8;
    Eigen::MatrixXi m_F, m_F2, m_F3, m_F4, m_F5, m_F6;
    Eigen::MatrixXd converged, converged2, converged3, converged4, converged5, converged6;

    Eigen::MatrixXd m_FN;
    Eigen::MatrixXd m_VN;
    Eigen::MatrixXd m_VN2;
    Eigen::MatrixXd m_VN_flann;


    int m_num_vex;
    float nv_len;
    float point_size;
    float line_width;
    int sel_vidx;
    int mode;
    int iterations;
    float max_distance;
    float percentage_volume;
    float z_rotation;
    float percentage_sample_1;
    float percentage_sample_2;
    ICP * icp;
    bool point_to_plane;

    void reset_display(igl::opengl::glfw::Viewer& viewer);
    void runICP(igl::opengl::glfw::Viewer& viewer);
    void changeMaxDistance(double value);
    void changeVolumePercentage(double value);
  private:
    void globalAlignment();

};

#endif
