#include "KdTree.h"
#include "ClosestPoint.h"

#include <algorithm>
#include <iostream>

KdTree::KdTree(const std::shared_ptr<Eigen::MatrixXd>& V)
{
	root = nullptr;
	nearest = nullptr;
	nodes.reserve(V->rows());
	max_dist = 0;

	for(size_t i = 0; i < V->rows(); i++)
		nodes.emplace_back(V->row(i), i);

	// Construct the tree
	root = makeKdTree(0, nodes.size(), 0);
}

KdTree::~KdTree()
{
}

KdNode* KdTree::makeKdTree(size_t start, size_t end, size_t dimension)
{
	if(start >= end)
		return nullptr;

	size_t mid = start + (end - start)/2;
	std::nth_element(&nodes[start], &nodes[mid], &nodes[end], node_cmp(dimension));
	dimension = (dimension + 1) % DIMENSIONS;
	nodes[mid].left = makeKdTree(start, mid, dimension);
	nodes[mid].right = makeKdTree(mid + 1, end, dimension);

	return &nodes[mid];
}

bool KdTree::closestPoint(const Eigen::Vector3d& queryPoint, const float maxDist, Eigen::Vector3d& point, int& index)
{
	nearest = nullptr;
	max_dist = 0;

	// Tree is empty
	if(root == nullptr) {
		std::cerr << "KdTree is empty, construct it first\n";
		return false;
	}

	// Search the KdTree
	findNearest(root, queryPoint, 0);
	point = nearest->vertex;
	index = nearest->index;

	// If point found is more than the maximum distance then we need to return false.
	if(ClosestPoint::euclideanDistance(point, queryPoint) > maxDist)
		return false;

	return true;
}

void KdTree::findNearest(KdNode* root, const Eigen::Vector3d& point, size_t dimension)
{
	if(root ==nullptr)
		return;

	float dist = ClosestPoint::euclideanDistance(root->vertex, point);
	if(nearest == nullptr || dist < max_dist) {
		max_dist = dist;
		nearest = root;
	}

	if(max_dist == 0)
		return;

	float dx = root->vertex(dimension) - point(dimension);
	dimension = (dimension + 1) % DIMENSIONS;
	findNearest(dx > 0 ? root->left : root->right, point, dimension);

	if(abs(dx) >= max_dist)
		return;
	findNearest(dx > 0 ? root->right : root->left, point, dimension);
}
