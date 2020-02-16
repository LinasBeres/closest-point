#ifndef TEST_H
#define TEST_H

#include "ClosestPoint.h"
#include "Context.h"
#include <random>

void runTests(Context context)
{
	const int n_tests = 500;

	ClosestPoint finder(context.getVertices());
	finder.constructKdTree();

	// Bounding box
	const Eigen::Vector3d m = context.getVertices()->colwise().minCoeff();
	const Eigen::Vector3d M = context.getVertices()->colwise().maxCoeff();

	std::default_random_engine generator;
	std::uniform_real_distribution<float> distributionX(m(0), M(0));
	std::uniform_real_distribution<float> distributionY(m(1), M(1));
	std::uniform_real_distribution<float> distributionZ(m(2), M(2));

	const float dist = ClosestPoint::euclideanDistance(m, M);
	std::uniform_real_distribution<float> distributionDist(0, dist);

	// Find point using all three methods and make sure that they all output the same point and index if found
	int passed = 0;
	for(int i = 0; i < n_tests; i++) {
		const Eigen::Vector3d query_point(distributionX(generator), distributionY(generator), distributionZ(generator));
		const float maxDist = distributionDist(generator);

		int indexBrute = 0, indexThreaded = 0, indexKdTree = 0;

		Eigen::Vector3d outBrute(0,0,0);
		Eigen::Vector3d outThreaded(0,0,0);
		Eigen::Vector3d outKdTree(0,0,0);

		bool foundBrute = finder.closestPointBruteForce(query_point, maxDist, outBrute, indexBrute);
		bool foundThreaded = finder.closestPointThreaded(query_point, maxDist, outThreaded, indexThreaded);
		bool foundKdTree = finder.closestPointKdTree(query_point, maxDist, outKdTree, indexKdTree);

		if(foundBrute == foundThreaded && foundBrute == foundKdTree && foundThreaded == foundKdTree) {
			if(foundBrute) {
				if(indexBrute == indexThreaded && indexBrute == indexKdTree && indexThreaded == indexKdTree) {
					if(outBrute == outThreaded && outBrute == outKdTree && outThreaded == outKdTree) {
						passed++;
					}
				}
			} else {
				passed++;
			}
		}
	}

	std::cerr << "Passed: " << passed << " out of " << n_tests << " point queries.\n";
};

#endif
