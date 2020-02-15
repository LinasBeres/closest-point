#include "Context.h"

#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/file_exists.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
	if (key == 'q' || key == 'Q') {
		exit(0);
	}
  return false;
}

Context::Context()
{
	viewer.callback_key_down = &key_down;
}

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
	viewer.data().set_mesh(*V, *F);
	viewer.core().align_camera_center(*V, *F);
	viewer.launch();
}
