#ifndef KDTREE_POINT_H
#define KDTREE_POINT_H

#include "ClosestPoint.h"

#include <Eigen/Eigen>
#include <memory>
#include <vector>

class KdNode
{
	public:
		KdNode(const Eigen::Vector3d& vertex, const size_t index) : vertex(vertex), index(index), left(nullptr), right(nullptr) {}
		~KdNode() {}

		Eigen::Vector3d vertex;
		size_t index;
		KdNode* left;
		KdNode* right;
};

class KdTree
{
	public:
		KdTree(const std::shared_ptr<Eigen::MatrixXd>& V);
		~KdTree();

		size_t treeSize()
		{
			return nodes.size();
		}

		bool closestPoint(const Eigen::Vector3d& queryPoint, const float maxDist, Eigen::Vector3d& point);
	private:
		struct node_cmp
		{
			node_cmp(size_t dimension) : dimension(dimension) {}
			bool operator()(const KdNode& n1, const KdNode& n2) const
			{
				return n1.vertex(dimension) < n2.vertex(dimension);
			}
			size_t dimension;
		};

		KdNode* makeKdTree(size_t start, size_t end, size_t dimension);
		void findNearest(KdNode* root, const Eigen::Vector3d& point, size_t dimension);

		KdNode* root;
		KdNode* nearest;
		std::vector<KdNode> nodes;
		float max_dist;
};


#endif
