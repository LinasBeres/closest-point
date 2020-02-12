#include "ICP.hpp"
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <limits>
#include <numeric>
#include <random>
#include <chrono>
#include <unordered_map>

using namespace std;
using namespace std::chrono;
using namespace Eigen;

//{{{ Class initialiser
ICP::ICP(MatrixXd point_cloud_1, MatrixXd point_cloud_2):
  point_cloud_1( point_cloud_1 ),
  point_cloud_2( point_cloud_2 )
{
  translate.setZero();
  rotation.setIdentity();
  max_distance = MAX_DISTANCE_DEFAULT;
  percentage_volume = 0.0;
  percentage_sample_1 = 1.0;
  percentage_sample_2 = 2.0;
  sample_1 = false;
  sample_2 = false;
  point_to_plane = false;
  point_cloud_3 = point_cloud_4 = point_cloud_5 = point_cloud_6  = MatrixXd::Identity(2,2);
}
///}}}

//{{{ Class Destructor
ICP::~ICP()
{
}
//}}}

//{{{ The Basic ICP Algorithm is implmented bellow
//--------------------------------------------------

//{{{ Get closest points of one scan to the other
list<pair<int, int>> ICP::getNeighbors(const MatrixXd src, const MatrixXd dst, double & error)
{
  list<pair<int, int>> point_pairs;
  const size_t par_k = 1; /* Nearest samples for the kdTree */
  vector<size_t> indexes(par_k);
  vector<double> dist_sqr(par_k);
  nanoflann::KNNResultSet<double> res(par_k);

  /* Reset error. */
  error = 0;

  /* First build the kdTree. */
  my_kd_tree_t mat_index(dst, MAX_LEAF);
  mat_index.index->buildIndex();
  res.init(indexes.data(), dist_sqr.data());

  for(int i = 0; i < src.rows(); i++) {
    res.init(indexes.data(), dist_sqr.data());
    RowVector3d item(src.row(i)(0), src.row(i)(1), src.row(i)(2));
    // item(0) += 0.0001; item(1) += 0.0001; item(2) += 0.0001;
    mat_index.index->findNeighbors(res, item.data(), nanoflann::SearchParams(MAX_LEAF));

    double dist = sqrt(pow(src.row(i)(0) - dst.row(indexes[0])(0), 2.0) +
        pow(src.row(i)(1) - dst.row(indexes[0])(1), 2.0) +
        pow(src.row(i)(2) - dst.row(indexes[0])(2), 2.0));

    if(dist <= max_distance) {
      error += dist;
      point_pairs.emplace_back(i, indexes[0]);
    }
  }

  error /= point_pairs.size();

  return point_pairs;
}
//}}}

MatrixXd ICP::icp(const int max_iterations)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  MatrixXd src, dst, dst_sample, src_sample;
  Vector3d average_p;
  list<pair<int, int>> point_pairs;
  double error, prev_error;
  bool end = false;

  cout << endl << "Max number of iterations: " << max_iterations << endl;
  cout << "Max distance between points: " << max_distance << endl;
  cout << "Point cloud 1: " << point_cloud_1.rows() << "x" << point_cloud_1.cols() << endl;
  cout << "Point cloud 2: " << point_cloud_2.rows() << "x" << point_cloud_2.cols() << endl;
  if(sample_1)
    cout << "Size of sample 1: " << point_cloud_1.rows()*percentage_sample_1 << endl;
  if(sample_2)
    cout << "Size of sample 2: " << point_cloud_2.rows()*percentage_sample_2 << endl;
  if(point_to_plane)
    cout << "Running point to plane" << endl;
  else
    cout << "Running point to point" << endl;

  /* Noise and sampling. */
  MatrixXd point_cloud_2_noise = addNoise(point_cloud_2);
  dst_sample = sample_1 ? createSample(point_cloud_1.transpose(), point_cloud_1.rows()*percentage_sample_1) : point_cloud_1.transpose();
  src = point_cloud_2_noise.transpose();
  dst = point_cloud_1.transpose();

  error = prev_error = std::numeric_limits<double>::min();

  /* Iterate and align. */
  for(int i = 0; i < max_iterations; i++) {
    /* Get sample if sampling turned on. */
    src_sample = sample_2 ? createSample(src, src.cols()*percentage_sample_2) : src;

    /* First we get closest point in dst for every point in our src. */
    prev_error = error;
    point_pairs = getNeighbors(src_sample.transpose(), dst_sample.transpose(), error);
    // cout << error << " ";

    /* Change in error is tiny. */
    if(abs(error - prev_error) <= TOL && !end) {
      high_resolution_clock::time_point t3 = high_resolution_clock::now();
      auto duration1 = duration_cast<milliseconds>( t3 - t1 ).count();

      cout << "TIME: " << duration1 << endl;
      cout << "EXITING AT: " << i-1 << endl;
      cout << "CURRENT ERROR: " << error << endl;
      end = true;
      // break;
    }

    if(point_to_plane)
      pointToPlane(src_sample, dst_sample, point_pairs);
    else
      pointToPoint(src_sample, dst_sample, point_pairs);

    /* Scan alignment. */
    src = rotation * src;
    src = src.colwise() + translate;
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>( t2 - t1 ).count();

  cout << endl;
  cout << "Finished running ICP, taking time: " << duration << endl;

  return src.transpose();
}

//}}}

//{{{ Point to Plane
void ICP::pointToPlane(const MatrixXd src_sample, const MatrixXd dst_sample, const list<pair<int,int>> point_pairs)
{
  Vector3d n, s, d;
  double a_1, a_2, a_3;
  VectorXd b(point_pairs.size()), point_diff;
  MatrixXd A(point_pairs.size(), 6), U, V, Vt;
  int j = 0;

  for(auto & x: point_pairs) {
    n = m_VN.col(x.second);
    s = src_sample.col(x.first);
    d  = dst_sample.col(x.second);

    /* Calculate b. */
    point_diff = d - s;
    b(j) = point_diff.dot(n);

    /* Calculate vector cross. */
    a_1 = n(2)*s(1) - n(1)*s(2);
    a_2 = n(0)*s(2) - n(2)*s(0);
    a_3 = n(1)*s(0) - n(0)*s(1);
    A.row(j) << a_1, a_2, a_3, n(0), n(1), n(2);

    j++;
  }

  /* Compute SVD. */
  JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
  auto singular_values = svd.singularValues();
  U = svd.matrixU();
  V = svd.matrixV();
  Vt = V.transpose();

  /* Invert the singular matrix. */
  double pinvtoler = 1.e-6; // choose your tolerance wisely!
  auto singularValues_inv = singular_values;
  for ( long k=0; k< singular_values.rows(); ++k) {
    if ( singular_values(k) > pinvtoler )
      singularValues_inv(k)=1.0/singular_values(k);
    else singularValues_inv(k)=0;
  }

  /* Get x from doing the pseudo inverse of A. */
  MatrixXd pinvmat = (V * singularValues_inv.asDiagonal() * U.transpose());
  VectorXd x = pinvmat*b;

  /* Translate vector. */
  translate << x(3), x(4), x(5);

  /* Calculate the rotation matrices. */
  MatrixXd Rx(3,3), Ry(3,3), Rz(3,3);

  Rx << 1, 0, 0,
     0, cos(x(0)), -sin(x(0)),
     0, sin(x(0)), cos(x(0));

  Ry << cos(x(1)), 0, sin(x(1)),
     0, 1, 0,
     -sin(x(1)), 0, cos(x(1));

  Rz << cos(x(2)), -sin(x(2)), 0,
     sin(x(2)), cos(x(2)), 0,
     0, 0, 1;

  /* Calculate the rotation matrix. */
  rotation = Rx*Ry*Rz;
}

//}}}

//{{{ Point to Point
void ICP::pointToPoint(
    const MatrixXd src_sample,
    const MatrixXd dst_sample,
    const list<pair<int, int>> point_pairs)
{
  Vector3d central_p, central_q;
  MatrixXd A, U, V, Vt;

  /* Compute average of src. */
  Vector3d average_q = Vector3d::Zero();
  Vector3d average_p = Vector3d::Zero();
  for(auto & x: point_pairs) {
    average_q += src_sample.col(x.first);
    average_p += dst_sample.col(x.second);
  }
  average_q /= point_pairs.size();
  average_p /= point_pairs.size();

  /* Recenter and compute A. */
  A = MatrixXd::Zero(3,3);
  for(auto & x: point_pairs) {
    central_p = dst_sample.col(x.second) - average_p;
    central_q = src_sample.col(x.first) - average_q;
    A += central_q * central_p.transpose();            /* XXX: central_q * central_p.transpose() seems to work */
  }

  /* Compute SVD. */
  JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
  U = svd.matrixU();
  V = svd.matrixV();
  Vt = V.transpose();

  /* Compute rotation and translation. */
  rotation = V * U.transpose();

  /* This seems not to make a difference? */
  if(rotation.determinant() < 0) {
    V.col(2) *= -1;
    rotation = V * U.transpose();
  }

  /* Calculate translate vector. */
  translate = average_p - rotation*average_q;
}

//}}}

//{{{ Create Sample
/**
 * Create sample from uniform distribution.
 */

MatrixXd ICP::createSample(const MatrixXd point_cloud, const int number_sample)
{
  MatrixXd out;
  out.resize(3, number_sample);
  default_random_engine generator;
  uniform_int_distribution<int> distribution(0,point_cloud.cols());

  for(int i = 0; i < number_sample; i++) {
    int index = distribution(generator);
    out.col(i) = point_cloud.col(index);
  }
  return out;
}

//}}}

//{{{ Add Noise

MatrixXd ICP::addNoise(MatrixXd point_cloud)
{
  MatrixXd out;
  RowVector3d rnd_vector;
  out.resize(point_cloud.rows(), 3);
  std::default_random_engine generator;

  Vector3d max_point, min_point;
  calculateBoundingBox(point_cloud, max_point, min_point);

  auto diff = max_point - min_point;
  double stddev = sqrt(pow(diff(0),2.0) + pow(diff(1),2.0) + pow(diff(2),2.0))*percentage_volume;
  std::normal_distribution<double> dist(0.0, stddev);

  double x_dim_sttdev = abs(max_point(0) - min_point(0))*percentage_volume;
  double y_dim_sttdev = abs(max_point(1) - min_point(1))*percentage_volume;
  double z_dim_sttdev = abs(max_point(2) - min_point(2))*percentage_volume;
  // cout << "X Direction stddev: " << x_dim_sttdev << endl;
  // cout << "Y Direction stddev: " << y_dim_sttdev << endl;
  // cout << "Z Direction stddev: " << z_dim_sttdev << endl;

  std::normal_distribution<double> dist_x(0.0, x_dim_sttdev);
  std::normal_distribution<double> dist_y(0.0, y_dim_sttdev);
  std::normal_distribution<double> dist_z(0.0, z_dim_sttdev);

  for(int i=0; i < point_cloud.rows(); i++) {
    rnd_vector = RowVector3d(dist_x(generator), dist_y(generator), dist_z(generator));
    out.row(i) = point_cloud.row(i) + rnd_vector;
  }

  return out;
}

//}}}

//{{{ Calculate Bounding Box

void ICP::calculateBoundingBox(const MatrixXd point_cloud, Vector3d & max_point, Vector3d & min_point)
{
  RowVector3d point;
  double min_z, min_y, min_x, max_z, max_y, max_x;
  min_z = min_y = min_x = numeric_limits<double>::max();
  max_z = max_y = max_x = numeric_limits<double>::min();

  for(int i = 0; i < point_cloud.rows(); i++) {
    point = point_cloud_2.row(i);
    max_z = point(2) >= max_z ? point(2) : max_z;
    max_y = point(1) >= max_y ? point(1) : max_y;
    max_x = point(0) >= max_x ? point(0) : max_x;

    min_z = point(2) <= min_z ? point(2) : min_z;
    min_y = point(1) <= min_y ? point(1) : min_y;
    min_x = point(0) <= min_x ? point(0) : min_x;
  }

  min_point = Vector3d(min_x, min_y, min_z);
  max_point = Vector3d(max_x, max_y, max_z);
}

///}}}

//{{{ Get and Put methods
Vector3d ICP::getTranslation()
{
  return translate;
}

MatrixXd ICP::getRotation()
{
  return rotation;
}

void ICP::changeMaxDistance(double new_max)
{
  max_distance = new_max;
}

void ICP::changeVolumePercentage(double new_percentage)
{
  percentage_volume = new_percentage;
}

void ICP::changeSample1(double new_sample)
{
  percentage_sample_1 = new_sample;
}

void ICP::changeSample2(double new_sample)
{
  percentage_sample_2 = new_sample;
}
//}}}
