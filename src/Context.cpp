#include "Context.h"

#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/file_exists.h>

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
	if (key == 'q' || key == 'Q') {
		exit(0);
	}
  return false;
}

Context::Context()
{
	display_query.resize(1, 3);
	display_point.resize(1, 3);
	maxDist = 0;
	query_point << 0,0,0;
	display_query.row(0) = query_point;
	x = 0, y = 0, z = 0;

	viewer.callback_key_down = &key_down;
	menu.callback_draw_custom_window = [&]()
	{
		// Define next window position + size
    ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(350, 225), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("Closest Point", nullptr, ImGuiWindowFlags_NoSavedSettings );

		if (ImGui::Button("Build KdTree", ImVec2(-1, 0))) {
			std::cout << "Constructing KdTree\n";
			finder->constructKdTree();
		}
		if(ImGui::InputFloat("maximum distance", &maxDist)) {
			std::cout << "Max distance changed to: " << maxDist << "\n";
		}
		if(ImGui::InputFloat("x position", &x)) {
			query_point(0) = x;
			resetDisplay();
		}
		if(ImGui::InputFloat("y position", &y)) {
			query_point(1) = y;
			resetDisplay();
		}
		if(ImGui::InputFloat("z position", &z)) {
			query_point(2) = z;
			resetDisplay();
		}
		if (ImGui::Button("Find Closest Point Brute Force", ImVec2(-1, 0))) {
			closestPoint(Mode::brute_force);
		}
		if (ImGui::Button("Find Closest Point Threaded", ImVec2(-1, 0))) {
			closestPoint(Mode::threaded);
		}
		if (ImGui::Button("Find Closest Point KdTree", ImVec2(-1, 0))) {
			closestPoint(Mode::kdtree);
		}

		ImGui::End();
	};
	viewer.plugins.push_back(&menu);

}

void Context::resetDisplay()
{
	viewer.data().clear();
	viewer.data().set_mesh(*V, *F);
	viewer.core().align_camera_center(*V, *F);
	display_query.row(0) = query_point;
	viewer.data().add_points(display_query, Eigen::RowVector3d(0, 0.9, 0));
}

bool Context::addMesh(const std::string& filepath)
{
	if(!igl::file_exists(filepath)) {
		std::cout << "File " << filepath << " does not exist.\n";
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

	finder = new ClosestPoint(V);

	viewer.launch();
}

void Context::closestPoint(Mode mode)
{
	bool found = false;
	Eigen::Vector3d out(0,0,0);
	int index = 0;

	if(mode == Mode::threaded) {
		std::cout << "Geting closest point threaded\n";
		found = finder->closestPointThreaded(query_point, maxDist, out, index);
	} else if(mode == Mode::kdtree) {
		std::cout << "Getting closest point with KdTree\n";
		found = finder->closestPointKdTree(query_point, maxDist, out, index);
	} else if(mode == Mode::brute_force) {
		std::cout << "Geting closest point non threaded\n";
		found = finder->closestPointBruteForce(query_point, maxDist, out, index);
	}

	display_point.row(0) = out;
	resetDisplay();
	if(found) {
		std::cout << "Closest Point: " << out(0) << "," << out(1) << "," << out(2) << "\n";
		std::cout << "Index: " << index << "\n";
		viewer.data().add_points(display_point, Eigen::RowVector3d(0, 0, 0.9));
	} else {
		std::cout << "Could not find point within maximum distance\n";
	}
}
