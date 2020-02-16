#include <iostream>

#include "test.h"
#include "Context.h"
#include "ClosestPoint.h"

int main(int argc, char *argv[])
{
	std::string mesh;
	Eigen::Vector3d query_point;
	float maxDist;
	bool command_line = false;
	Mode mode = Mode::brute_force;
	bool test = false;

	// Arguments
	for(int i = 1; i < argc; i++) {
		if(strcmp(argv[i], "--mesh") == 0) {
			if(i+1 >= argc) {
				std::cerr << "Please specify a mesh\n";
				return 1;
			}
			mesh = argv[i+1];
		} else if(strcmp(argv[i], "--point") == 0) {
			if(i+3 >= argc) {
				std::cerr << "Please specify a 3d point\n";
				return 1;
			}
			try {
				float x = std::stof(argv[i+1]);
				float y = std::stof(argv[i+2]);
				float z = std::stof(argv[i+3]);
				query_point << x, y, z;
			} catch(const std::invalid_argument& e) {
				std::cerr << "Please input a valid vector\n";
				return 1;
			}
		} else if(strcmp(argv[i], "--dist") == 0) {
			if(i+1 >= argc) {
				std::cerr << "Please specify a maximum distance\n";
				return 1;
			}
			try {
				maxDist = std::stof(argv[i+1]);
			} catch(const std::invalid_argument& e) {
				std::cerr << "Please input a valid maximum distance\n";
				return 1;
			}
		} else if(strcmp(argv[i], "--cmd") == 0) {
			command_line = true;
		} else if(strcmp(argv[i], "--threaded") == 0) {
			mode = Mode::threaded;
		} else if(strcmp(argv[i], "--kdtree") == 0) {
			mode = Mode::kdtree;
		} else if(strcmp(argv[i], "--test") == 0) {
			test = true;
		}
	}

	// A 'hack' to disable annoying libigl Viewer usage message.
	if(command_line || test)
		std::cout.setstate(std::ios::failbit);

	Context my_context;

	// Enable again
	if(command_line || test)
		std::cout.clear();

	if(!my_context.addMesh(mesh))
		return 1;

	if(test) {
		runTests(my_context);
		return 0;
	} else if(command_line) {
		bool found = false;
		int index;
		Eigen::Vector3d out;
		ClosestPoint finder(my_context.getVertices());

		std::cout << "In: " << query_point(0) << "," << query_point(1) << "," << query_point(2) << "\n";
		std::cout << "Maximum distance: " << maxDist << "\n";

		if(mode == Mode::threaded) {
			std::cout << "Geting closest point threaded\n";
			found = finder.closestPointThreaded(query_point, maxDist, out, index);
		} else if(mode == Mode::kdtree) {
			finder.constructKdTree();
			std::cout << "Getting closest point using KdTree\n";
			found = finder.closestPointKdTree(query_point, maxDist, out, index);
		} else {
			std::cout << "Geting closest point non threaded\n";
			found = finder.closestPointBruteForce(query_point, maxDist, out, index);
		}

		if(found) {
			std::cout << "Closest Point: " << out(0) << "," << out(1) << "," << out(2) << "\n";
			std::cout << "Index: " << index << "\n";
		} else {
			std::cout << "Could not find point within maximum distance\n";
		}
	} else {
		// Launch GUI
		my_context.display();
	}

	return 0;
}
