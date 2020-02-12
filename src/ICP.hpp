#ifndef ICP_H
#define ICP_H

#include <vector>
#include <utility>
#include <list>
#include <iostream>
#include <Eigen/Eigen>

#include "nanoflann.hpp"
#include "debug.h"

#define TOL 1e-6
#define MAX_LEAF 10
#define MAX_DISTANCE_DEFAULT 1

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> my_kd_tree_t;
// typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, nanoflann::metric_L2> my_kd_tree_t;
// typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, nanoflann::metric_L1> my_kd_tree_t;
// typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, nanoflann:metric_l2_Simple> my_kd_tree_t;

class ICP {
  public:
    ICP(Eigen::MatrixXd point_cloud_1, Eigen::MatrixXd point_cloud_2);
    ~ICP();

    Eigen::MatrixXd icp(const int max_iterations);

    /* Set & get methods. */
    void changeMaxDistance(double new_max);
    void changeVolumePercentage(double new_percentage);
    void changeSample1(double new_sample);
    void changeSample2(double new_sample);
    Eigen::Vector3d getTranslation();
    Eigen::MatrixXd getRotation();

    Eigen::MatrixXd m_VN;
    bool point_to_plane;

  private:
    /* Routine to get the rotation and translate using point to plane. */
    void pointToPlane(
        const Eigen::MatrixXd src_sample,
        const Eigen::MatrixXd dst_sample,
        const std::list<std::pair<int,int>> point_pairs);

    /* Routine to get the rotation and translate using point to point. */
    void pointToPoint(
        const Eigen::MatrixXd src_sample,
        const Eigen::MatrixXd dst_sample,
        const std::list<std::pair<int, int>> point_pairs);

    std::list<std::pair<int, int>> getNeighbors(const Eigen::MatrixXd src, const Eigen::MatrixXd dst, double & error);
    Eigen::MatrixXd addNoise(Eigen::MatrixXd point_cloud);
    void calculateBoundingBox(const Eigen::MatrixXd point_cloud, Eigen::Vector3d & max_point, Eigen::Vector3d & min_point);
    Eigen::MatrixXd createSample(const Eigen::MatrixXd point_cloud, const int number_sample);

    Eigen::MatrixXd point_cloud_1;
    Eigen::MatrixXd point_cloud_2;
    Eigen::MatrixXd point_cloud_3;
    Eigen::MatrixXd point_cloud_4;
    Eigen::MatrixXd point_cloud_5;
    Eigen::MatrixXd point_cloud_6;
    Eigen::Vector3d translate;
    Eigen::Matrix3d rotation;

    double max_distance;
    double percentage_volume;
    double percentage_sample_1;
    double percentage_sample_2;
    bool sample_1;
    bool sample_2;
};


#endif
