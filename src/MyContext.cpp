#include "MyContext.hpp"

//{{{ Class Constructor
MyContext::MyContext():
  nv_len(0.5),
  point_size(8),
  line_width(0.5),
  sel_vidx(7),
  mode(0)
{
  igl::readPLY("../data/bun315.ply", m_V, m_F);
  igl::readPLY("../data/bun000.ply", m_V2, m_F);
  igl::readPLY("../data/bun045.ply", m_V3, m_F2);
  igl::readPLY("../data/bun090.ply", m_V4, m_F3);
  igl::readPLY("../data/bun180.ply", m_V5, m_F4);
  igl::readPLY("../data/bun270.ply", m_V6, m_F5);

  igl::readPLY("../data/chin.ply", m_V7, m_F6);
  igl::readPLY("../data/top2.ply", m_V8, m_F6);

  igl::readPLY("../data/bun000.ply", m_V, m_F);
  igl::readPLY("../data/bun045.ply", m_V2, m_F2);

  calculate_vertex_normal_flann(m_V, m_VN);
  calculate_vertex_normal_flann(m_V2, m_VN2);

  converged = m_V2;
  converged2 = m_V6;
  converged3 = m_V3;
  converged4 = m_V4;
  converged5 = m_V5;

  m_num_vex = m_V.rows();

  std::cout << "SIZE OF FACES: " << m_F.cols() << "x" << m_F.rows() << std::endl;

  // calculate VN
  igl::per_face_normals(m_V, m_F, m_FN);

  iterations = 0;
  max_distance = 1.0;
  z_rotation = 0.0;
  percentage_volume = 0.0;
  percentage_sample_1 = 1.0;
  percentage_sample_2 = 1.0;
  point_to_plane = true;
}
//}}}

//{{{ Class Destructor
MyContext::~MyContext()
{
  free(icp);
}
//}}}

//{{{ Buttons
void MyContext::reset_display(igl::opengl::glfw::Viewer& viewer)
{
  viewer.data().clear();

  if (mode == 0) {
    viewer.data().add_points(m_V, Eigen::RowVector3d(0, 0, 1));
    viewer.core.align_camera_center(m_V);
  } else if(mode == 1) {
    viewer.data().add_points(converged, Eigen::RowVector3d(0, 1, 0));
    viewer.core.align_camera_center(converged);
  } else if(mode == 2) {
    viewer.data().add_points(m_V, Eigen::RowVector3d(0, 0, 1));
    viewer.data().add_points(converged, Eigen::RowVector3d(0, 0.9, 0));
    // viewer.data().add_points(converged2, Eigen::RowVector3d(0, 0.8, 0));
    // viewer.data().add_points(converged3, Eigen::RowVector3d(0, 0.7, 0));
    // viewer.data().add_points(converged4, Eigen::RowVector3d(0, 0.6, 0));
    // viewer.data().add_points(converged5, Eigen::RowVector3d(0, 0.5, 0));
    viewer.core.align_camera_center(m_V);
  } else if(mode == 3) {
    viewer.data().add_points(m_V, Eigen::RowVector3d(0, 0, 1));
    viewer.data().add_points(m_V2, Eigen::RowVector3d(1, 0, 0));
    viewer.core.align_camera_center(m_V);
    viewer.data().show_overlay = 1;
  }



  viewer.data().line_width = line_width;
  viewer.data().point_size = point_size;

}

void MyContext::runICP(igl::opengl::glfw::Viewer& viewer)
{

  Eigen::Matrix3d rotationx = getRotationX(z_rotation);
  Eigen::Matrix3d rotationy = getRotationY(z_rotation);
  Eigen::Matrix3d rotationz = getRotationZ(z_rotation);

  // m_V2 = rotationz * m_V.transpose();
  // m_V2.transposeInPlace();

  icp = new ICP(m_V, m_V2);

  icp->m_VN = m_VN.transpose();
  icp->point_to_plane = point_to_plane;

  icp->changeMaxDistance(max_distance);
  icp->changeVolumePercentage(percentage_volume);
  icp->changeSample1(percentage_sample_1);
  icp->changeSample2(percentage_sample_2);
  converged = icp->icp(iterations);
  // globalAlignment();

}

void MyContext::globalAlignment()
{
  Eigen::Matrix3d rotationx = getRotationX(z_rotation);
  Eigen::Matrix3d rotationy = getRotationY(z_rotation);
  Eigen::Matrix3d rotationz = getRotationZ(z_rotation);


  icp = new ICP(m_V, m_V2);

  icp->m_VN = m_VN.transpose();
  icp->point_to_plane = point_to_plane;

  icp->changeMaxDistance(max_distance);
  icp->changeVolumePercentage(percentage_volume);
  icp->changeSample1(percentage_sample_1);
  icp->changeSample2(percentage_sample_2);
  converged = icp->icp(iterations);
  // free(icp);

  icp = new ICP(m_V, m_V6);
  icp->m_VN = m_VN.transpose();
  icp->point_to_plane = point_to_plane;
  icp->changeMaxDistance(max_distance);
  icp->changeVolumePercentage(percentage_volume);
  icp->changeSample1(percentage_sample_1);
  icp->changeSample2(percentage_sample_2);
  converged2 = icp->icp(iterations);
  // free(icp);

  icp = new ICP(converged, m_V3);
  calculate_vertex_normal_flann(converged, m_VN);
  icp->m_VN = m_VN.transpose();
  icp->point_to_plane = point_to_plane;
  icp->changeMaxDistance(max_distance);
  icp->changeVolumePercentage(percentage_volume);
  icp->changeSample1(percentage_sample_1);
  icp->changeSample2(percentage_sample_2);
  converged3 = icp->icp(iterations);
  // free(icp);

  rotationy = getRotationY(90);
  test = (rotationy * m_V4.transpose());
  test.transposeInPlace();
  icp = new ICP(converged3, test);
  calculate_vertex_normal_flann(converged3, m_VN);
  icp->m_VN = m_VN.transpose();
  icp->point_to_plane = point_to_plane;
  icp->changeMaxDistance(max_distance);
  icp->changeVolumePercentage(percentage_volume);
  icp->changeSample1(percentage_sample_1);
  icp->changeSample2(percentage_sample_2);
  converged4 = icp->icp(iterations);
  // free(icp);

}

void MyContext::changeMaxDistance(double value)
{
  icp->changeMaxDistance(value);
}

void MyContext::changeVolumePercentage(double value)
{
  icp->changeVolumePercentage(value);
}
//}}}
