#ifndef KDTREE_POINT_H
#define KDTREE_POINT_H

#include "ClosestPoint.h"

#include <Eigen/Eigen>
#include <memory>
#include <vector>

class KdNode
{
	public:
		KdNode(const Eigen::Vector3d& vertex, const int index) : vertex(vertex), index(index), left(nullptr), right(nullptr) {}
		~KdNode() {}

		Eigen::Vector3d vertex;
		int index;
		KdNode* left;
		KdNode* right;
}

class KdTree
{
	public:
		KdTree();
		~KdTree();

		nearestPoint(const Eigen::Vector3d&);
	private:
		KdNode* makeKdTree();
		void findNearest();

		KdNode* root;
		KdNode* nearest;
		std::vector<KdNode> nodes;
		double maxDist;
};


#endif
