#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/writeOFF.h>
#include <igl/file_exists.h>
#include <imgui/imgui.h>
#include <iostream>
#include <random>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[])
{
	// Load a mesh in PLY format
	igl::readPLY("../data/bun000.ply", V, F);

	// Plot the mesh
	igl::opengl::glfw::Viewer viewer;
	viewer.data().clear();
	viewer.data().add_points(V, Eigen::RowVector3d(0, 0, 1));
	viewer.launch();

	return 0;
}
