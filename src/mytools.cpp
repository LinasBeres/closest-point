#include "mytools.h"
#include "nanoflann.hpp"

using namespace std;

//{{{ Rotation Matrices.
Eigen::Matrix3d getRotationZ(double theta)
{
  Eigen::Matrix3d rotation;
  double result;
  result = theta*M_PI/180;

  rotation << cos(result), -sin(result), 0,
              sin(result),  cos(result), 0,
                        0,            0, 1;

  return rotation;
}

Eigen::Matrix3d getRotationX(double theta)
{
  Eigen::Matrix3d rotation;
  double result;
  result = theta*M_PI/180;

  rotation << 1,           0,            0,
              0, cos(result), -sin(result),
              0, sin(result),  cos(result);

  return rotation;
}
Eigen::Matrix3d getRotationY(double theta)
{
  Eigen::Matrix3d rotation;
  double result;
  result = theta*M_PI/180;

  rotation <<  cos(result), 0, sin(result),
                         0, 1,           0,
              -sin(result), 0, cos(result);

  return rotation;
}
//}}}

//{{{ Calculate Normals
void calculate_vertex_normal_flann(Eigen::MatrixXd const & V, Eigen::MatrixXd & out_VN)
{
  out_VN.resize(V.rows(), V.cols());
  out_VN.setZero();

  typedef nanoflann::KDTreeEigenMatrixAdaptor< Eigen::MatrixXd >  my_kd_tree_t;

  my_kd_tree_t   mat_index(V, 50 /* max leaf */);
  mat_index.index->buildIndex();

  Eigen::RowVector3d v_cen = V.colwise().sum() / V.rows();

  for (int idx = 0; idx < V.rows(); idx++) {

    Eigen::RowVector3d i_vex = V.row(idx);

    const size_t num_results = 8;
    std::vector<size_t>   ret_indexes(num_results);
    std::vector<double> out_dists_sqr(num_results);

    nanoflann::KNNResultSet<double> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, i_vex.data() , nanoflann::SearchParams(25));

    Eigen::MatrixXd Selectpoints(num_results, 3);

    for (size_t i = 0; i < num_results; i++) {
      //std::cout << "ret_index["<<i<<"]=" << ret_indexes[i] << " out_dist_sqr=" << out_dists_sqr[i] << endl;
      Selectpoints(i, 0) = V(ret_indexes[i], 0);
      Selectpoints(i, 1) = V(ret_indexes[i], 1);
      Selectpoints(i, 2) = V(ret_indexes[i], 2);
    }

    Eigen::RowVector3d pl_NV, Ct;

    igl::fit_plane(Selectpoints, pl_NV, Ct);

    if ((v_cen - i_vex).dot(pl_NV)  > 0) {
      pl_NV = pl_NV * -1;
    }

    out_VN.row(idx) = pl_NV;

  }


}
//}}}
