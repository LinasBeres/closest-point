#ifndef CONTEXT_H
#define CONTEXT_H

#include "ClosestPoint.h"

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <Eigen/Eigen>
#include <string>
#include <memory>

enum class Mode {brute_force, threaded, kdtree};

class Context
{
	public:
		Context();
		~Context() {}

		// Loades in the vertices and faces into V & F, return false if file doesn't exist
		bool addMesh(const std::string& filepath);
		// Displays the GUI
		void display();


		std::shared_ptr<Eigen::MatrixXd> getVertices()
		{
			return V;
		};

	private:
		void closestPoint(Mode mode);

		float x,y,z;
		float maxDist;
		Eigen::Vector3d query_point;
		Eigen::MatrixXd display_query, display_point;
		ClosestPoint* finder;
		igl::opengl::glfw::Viewer viewer;
		igl::opengl::glfw::imgui::ImGuiMenu menu;
		std::shared_ptr<Eigen::MatrixXd> V;
		std::shared_ptr<Eigen::MatrixXi> F;
};


#endif
