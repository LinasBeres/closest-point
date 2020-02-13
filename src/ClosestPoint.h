#ifndef CLOSEST_POINT_H
#define CLOSEST_POINT_H


#include <Eigen/Eigen>
#include <memory>


class ClosestPoint
{
	public:
		ClosestPoint(const std::shared_ptr<Eigen::MatrixXd> V) : V(V) {}
		~ClosestPoint() {}

		bool closestPointBruteForce(const Eigen::Vector3d& queryPoint, float maxDist, Eigen::Vector3d& point) const;
		bool closestPointThreaded(const Eigen::Vector3d& queryPoint, float maxDist, Eigen::Vector3d& point) const;

		static float euclideanDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& q);
	private:

		std::shared_ptr<Eigen::MatrixXd> V;
};


#endif
