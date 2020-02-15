#include <iostream>

#include "Context.h"
#include "ClosestPoint.h"

int main(int argc, char *argv[])
{
	std::string mesh;
	Eigen::Vector3d query_point;
	float maxDist;
	bool command_line = false;
	bool threaded_mode = false;
	bool kdtree_mode = false;

	for(int i = 1; i < argc; i++) {
		if(strcmp(argv[i], "--mesh") == 0) {
			if(i+1 >= argc) {
				std::cout << "Please specify a mesh\n";
				return 1;
			}
			mesh = argv[i+1];
		} else if(strcmp(argv[i], "--point") == 0) {
			if(i+3 >= argc) {
				std::cout << "Please specify a 3d point\n";
				return 1;
			}
			query_point << std::stof(argv[i+1]), std::stof(argv[i+2]), std::stof(argv[i+3]);
		} else if(strcmp(argv[i], "--dist") == 0) {
			if(i+1 >= argc) {
				std::cout << "Please specify a maximum distance\n";
				return 1;
			}
			maxDist = std::stof(argv[i+1]);
		} else if(strcmp(argv[i], "--cmd") == 0) {
			command_line = true;
		} else if(strcmp(argv[i], "--threaded") == 0) {
			threaded_mode = true;
			kdtree_mode = false;
		} else if(strcmp(argv[i], "--kdtree") == 0) {
			kdtree_mode = true;
			threaded_mode = false;
		}
	}

	Context my_context;
	if(!my_context.addMesh(mesh))
		return 1;

	if(command_line) {
		bool found = false;
		int index;
		Eigen::Vector3d out;
		ClosestPoint finder(my_context.getVertices());

		std::cout << "\n";
		std::cout << "In: " << query_point(0) << "," << query_point(1) << "," << query_point(2) << "\n";
		std::cout << "Maximum distance: " << maxDist << "\n";

		if(threaded_mode) {
			std::cout << "Geting closest point threaded\n";
			found = finder.closestPointThreaded(query_point, maxDist, out, index);
		} else if(kdtree_mode) {
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
		my_context.display();
	}

	return 0;
}
