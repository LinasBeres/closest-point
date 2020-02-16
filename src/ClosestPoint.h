#ifndef CLOSEST_POINT_H
#define CLOSEST_POINT_H


#include "KdTree.h"
#include <Eigen/Eigen>
#include <memory>

class KdTree;

/**
 * Class for find the nearest point in a mesh using three different methods.
 */
class ClosestPoint
{
	public:
		ClosestPoint(const std::shared_ptr<Eigen::MatrixXd> V) : V(V), kdTree(nullptr) {}
		~ClosestPoint() {}

		bool constructKdTree();

		bool closestPointBruteForce(const Eigen::Vector3d& queryPoint, float maxDist, Eigen::Vector3d& point, int& index) const;
		bool closestPointThreaded(const Eigen::Vector3d& queryPoint, float maxDist, Eigen::Vector3d& point, int& index) const;
		bool closestPointKdTree(const Eigen::Vector3d& queryPoint, const float maxDist, Eigen::Vector3d& point, int& index) const;

		static float euclideanDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& q);
	private:
		KdTree* kdTree;
		std::shared_ptr<Eigen::MatrixXd> V;
};


#endif
