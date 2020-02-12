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

MyContext g_myctx;

int main(int argc, char *argv[])
{
  // Init the viewer
  igl::opengl::glfw::Viewer viewer;

  // Attach a menu plugin
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.plugins.push_back(&menu);

  // menu variable Shared between two menus
  double doubleVariable = 0.1f;


  // Add additional windows via defining a Lambda expression with captures by reference([&])
  menu.callback_draw_custom_window = [&]() {
    // Define next window position + size
    ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 300), ImGuiSetCond_FirstUseEver);
    ImGui::Begin( "MyProperties", nullptr, ImGuiWindowFlags_NoSavedSettings );


    ImGui::End();
  };


  // registered a event handler
  viewer.callback_key_down = &key_down;

  g_myctx.reset_display(viewer);

  // Call GUI
  viewer.launch();


  return 0;
};
