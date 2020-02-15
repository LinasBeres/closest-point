#ifndef CONTEXT_H
#define CONTEXT_H


#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Eigen>
#include <string>
#include <memory>


class Context
{
	public:
		Context();
		~Context() {}

		// Loades in the vertices and faces into V & F, return false if file doesn't exist
		bool addMesh(const std::string& filepath);
		// Displays the GUI
		void display();

		bool closestPoint(const Eigen::Vector3d& queryPoint, float maxDist, Eigen::Vector3d& point);

		std::shared_ptr<Eigen::MatrixXd> getVertices()
		{
			return V;
		};

	private:
		igl::opengl::glfw::Viewer viewer;
		std::shared_ptr<Eigen::MatrixXd> V;
		std::shared_ptr<Eigen::MatrixXi> F;
};


#endif
