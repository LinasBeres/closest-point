#include "Context.h"

#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/file_exists.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>


bool Context::addMesh(const std::string& filepath)
{
	if(!igl::file_exists(filepath)) {
		std::cerr << "File " << filepath << " does not exist.\n";
		return false;
	}

	Eigen::MatrixXd Vertices;
	Eigen::MatrixXi Faces;
	if(!igl::readPLY(filepath, Vertices, Faces))
		return false;

	V = std::make_shared<Eigen::MatrixXd>(Vertices);
	F = std::make_shared<Eigen::MatrixXi>(Faces);
	return true;
}

void Context::display()
{
	viewer.data().clear();
	viewer.data().add_points(*V, Eigen::RowVector3d(0, 0, 1));
	viewer.core().align_camera_center(*V, *F);
	viewer.launch();
}
