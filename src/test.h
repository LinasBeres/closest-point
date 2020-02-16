#ifndef TEST_H
#define TEST_H

#include "ClosestPoint.h"
#include "Context.h"
#include <random>
#include <chrono>

void runTests(Context context)
{
	const int n_tests = 500;

	ClosestPoint finder(context.getVertices());
	finder.constructKdTree();

	// Average time for building kdtree
	float durationBuild = 0;
	for(int i = 0; i < n_tests; i++) {
		ClosestPoint kdTest(context.getVertices());
		auto t1 = std::chrono::high_resolution_clock::now();
		kdTest.constructKdTree();
		auto t2 = std::chrono::high_resolution_clock::now();
		durationBuild += std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
	}
	std::cerr << "Average time for building KdTree: " << durationBuild / n_tests << " microseconds.\n";

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
	float durationBrute = 0, durationThreaded = 0, durationKdTree = 0;
	for(int i = 0; i < n_tests; i++) {
		const Eigen::Vector3d query_point(distributionX(generator), distributionY(generator), distributionZ(generator));
		const float maxDist = distributionDist(generator);

		int indexBrute = 0, indexThreaded = 0, indexKdTree = 0;

		Eigen::Vector3d outBrute(0,0,0);
		Eigen::Vector3d outThreaded(0,0,0);
		Eigen::Vector3d outKdTree(0,0,0);

		auto t1 = std::chrono::high_resolution_clock::now();
		bool foundBrute = finder.closestPointBruteForce(query_point, maxDist, outBrute, indexBrute);
		auto t2 = std::chrono::high_resolution_clock::now();
		durationBrute += std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

		t1 = std::chrono::high_resolution_clock::now();
		bool foundThreaded = finder.closestPointThreaded(query_point, maxDist, outThreaded, indexThreaded);
		t2 = std::chrono::high_resolution_clock::now();
		durationThreaded += std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

		t1 = std::chrono::high_resolution_clock::now();
		bool foundKdTree = finder.closestPointKdTree(query_point, maxDist, outKdTree, indexKdTree);
		t2 = std::chrono::high_resolution_clock::now();
		durationKdTree += std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

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

	std::cerr << "\n";
	std::cerr << "Search timings:\n";
	std::cerr << "Average time for brute force search: " << durationBrute / n_tests << " microseconds.\n";
	std::cerr << "Average time for threaded search:    " << durationThreaded / n_tests << " microseconds.\n";
	std::cerr << "Average time for KdTree search:      " << durationKdTree / n_tests << " microseconds.\n";
	std::cerr << "\n";

	std::cerr << "Passed: " << passed << " out of " << n_tests << " point queries.\n";
};

#endif
