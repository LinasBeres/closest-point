#include <iostream>

#include "Context.h"
#include "ClosestPoint.h"

int main(int argc, char *argv[])
{
	const std::string& mesh = argc > 1 ? argv[1] : "../data/bun000.ply";

	Context my_context;

	if(!my_context.addMesh(mesh))
		return 1;

	// my_context.display();
	ClosestPoint finder(my_context.getVertices());


	std::cerr << "Geting closest point threaded\n";
	Eigen::Vector3d v(-0.0075, 0.0344859, 0.0216591);
	Eigen::Vector3d w;
	finder.closestPointThreaded(v, 50, w);

	std::cerr << "w: " << w << "\n";

	std::cerr << "Geting closest point non threaded\n";
	Eigen::Vector3d p(-0.0075, 0.0344859, 0.0216591);
	Eigen::Vector3d q;
	finder.closestPointBruteForce(p, 50, q);

	std::cerr << "w: " << q << "\n";

	finder.constructKdTree();
	std::cerr << "Getting closest point using KdTree\n";
	Eigen::Vector3d s(-0.0075, 0.0344859, 0.0216591);
	Eigen::Vector3d t;
	finder.closestPointKdTree(s, 50, t);

	std::cerr << "w: " << t << "\n";

	return 0;
}
