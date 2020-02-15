#include "ClosestPoint.h"

#include <iostream>
#include <future>
#include <vector>
#include <tuple>

/**
 * Find closet point to search query given a maximum distance.
 * O(n)
 */
bool ClosestPoint::closestPointBruteForce(const Eigen::Vector3d& queryPoint, const float maxDist, Eigen::Vector3d& point, int& index) const
{
	float min_distance = maxDist;
	bool found = false;

	for(size_t i = 0; i < V->rows(); i++) {
		float dist = euclideanDistance(V->row(i), queryPoint);

		// Assuming that if the point queried is in the mesh then instead we want a new point
		if(dist == 0)
			continue;

		if(dist <= min_distance) {
			min_distance = dist;
			point = V->row(i);
			found = true;
			index = i;
		}
	}

	return found;
}

/**
 * Same as above but threaded.
 */
bool ClosestPoint::closestPointThreaded(const Eigen::Vector3d& queryPoint, const float maxDist, Eigen::Vector3d& point, int& index) const
{
	const size_t cores = std::thread::hardware_concurrency();
	std::vector<std::future<std::tuple<bool, Eigen::Vector3d, float, int>>> future_vector;

	// Split work into equal chunks of the vertices from the mesh
	// so that each core gets an equal amount of work
	const size_t chunk = V->rows() / cores + (V->rows() % cores != 0);

	// Construct the futures and return a tuple for each chunk
	for(size_t i = 0; i < cores; i++) {
		future_vector.emplace_back(std::async(std::launch::async, [=]() {
			auto out = std::make_tuple(false, Eigen::Vector3d(0,0,0), maxDist, 0);

			for(size_t j = i*chunk; j < chunk*(i+1) && j < V->rows(); j++) {
				float dist = euclideanDistance(V->row(j), queryPoint);

				if(dist == 0)
					continue;

				if(dist <= std::get<2>(out)) {
					std::get<0>(out) = true;
					std::get<1>(out) = V->row(j);
					std::get<2>(out) = dist;
					std::get<3>(out) = j;
				}
			}

			return out;
		}));
	}

	bool found = false;
	float min_distance = maxDist;

	// Get the promises and see if we have a winner
	for(auto &f : future_vector) {
		const auto out = f.get();

		if(std::get<0>(out) && std::get<2>(out) <= min_distance) {
			min_distance = std::get<2>(out);
			point = std::get<1>(out);
			found = true;
			index = std::get<3>(out);
		}
	}

	return found;
}

bool ClosestPoint::constructKdTree()
{
	kdTree = new KdTree(V);

	return kdTree->treeSize() != 0;
}

/**
 * Same but using the KdTree
 */
bool ClosestPoint::closestPointKdTree(const Eigen::Vector3d& queryPoint, const float maxDist, Eigen::Vector3d& point, int& index) const
{
	if(kdTree == nullptr || kdTree->treeSize() == 0) {
		std::cout << "KdTree has not been constructed, please construct it first.\n";
		return false;
	}

	return kdTree->closestPoint(queryPoint, maxDist, point, index);
}

float ClosestPoint::euclideanDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& q)
{
	float d1 = pow(p(0) - q(0), 2);
	float d2 = pow(p(1) - q(1), 2);
	float d3 = pow(p(2) - q(2), 2);

	return sqrt(d1 + d2 + d3);
}
