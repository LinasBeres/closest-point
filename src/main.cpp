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

#include "debug.h"
#include "MyContext.hpp"
#include "mytools.h"
#include "ICP.hpp"

using namespace std;

MyContext g_myctx;

//{{{ Key Down Function.
bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{

  std::cout << "Key: " << key << " " << (unsigned int)key << std::endl;
  if (key=='q' || key=='Q')
  {
    exit(0);
  }
  return false;
}
//}}}

//{{{ Main
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

    // point size
    // [event handle] if value changed
    if (ImGui::InputFloat("point_size", &g_myctx.point_size)) {
      std::cout << "point_size changed\n";
      viewer.data().point_size = g_myctx.point_size;
    }

    // line width
    // [event handle] if value changed
    if(ImGui::InputFloat("line_width", &g_myctx.line_width)) {
      std::cout << "line_width changed\n";
      viewer.data().line_width = g_myctx.line_width;
    }

    if(ImGui::InputFloat("max_distance", &g_myctx.max_distance)) {
      std::cout << "Max distance between points has changed to " << g_myctx.max_distance << endl;
    }

    if(ImGui::InputFloat("percentage_volume", &g_myctx.percentage_volume)) {
      std::cout << "Changed the percentage of volume to " << g_myctx.percentage_volume<< endl;
    }

    if(ImGui::InputFloat("percentage_sample_1", &g_myctx.percentage_sample_1)) {
      std::cout << "Changed the percentage of sample to " << g_myctx.percentage_sample_1 << endl;
    }

    if(ImGui::InputFloat("percentage_sample_2", &g_myctx.percentage_sample_2)) {
      std::cout << "Changed the percentage of sample to " << g_myctx.percentage_sample_2 << endl;
    }

    if(ImGui::InputFloat("z_rotation", &g_myctx.z_rotation)) {
      std::cout << "Z rotation has changed to " << g_myctx.z_rotation << endl;
    }

    if(ImGui::InputInt("max_iterations", &g_myctx.iterations)) {
      std::cout << "Number of iterations has changed to " << g_myctx.iterations << endl;
    }

    //mode
    if (ImGui::SliderInt("mode", &g_myctx.mode, 0,3)) {
      g_myctx.reset_display(viewer);
    }

    //mode-text
    if (g_myctx.mode==0) {
      ImGui::Text("Original Scan");
    } else if (g_myctx.mode == 1) {
      ImGui::Text("Allignment Scan");
    } else if(g_myctx.mode == 2) {
      ImGui::Text("Original + allignment");
    } else if(g_myctx.mode == 3){
      ImGui::Text("Original two scans");
    }

    if (ImGui::Button("Reset Display", ImVec2(-1, 0))) {
      g_myctx.reset_display(viewer);
    }
    if (ImGui::Button("Run ICP", ImVec2(-1, 0))) {
      g_myctx.runICP(viewer);
    }
    ImGui::End();
  };


  // registered a event handler
  viewer.callback_key_down = &key_down;

  g_myctx.reset_display(viewer);

  // Call GUI
  viewer.launch();


  return 0;
};
//}}}

//{{{ Main part of the menu currently not needed

  // Add content to the default menu window via defining a Lambda expression with captures by reference([&])
  // menu.callback_draw_viewer_menu = [&]() {
    // // Draw parent menu content
    // menu.draw_viewer_menu();
//
    // // Add new group
    // if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen)) {
      // // Expose variable directly ...
      // ImGui::InputDouble("double", &doubleVariable, 0, 0, "%.4f");
//
      // // ... or using a custom callback
      // static bool boolVariable = true;
      // if (ImGui::Checkbox("bool", &boolVariable)) {
        // // do something
        // std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
      // }
//
      // // Expose an enumeration type
      // enum Orientation { Up = 0, Down, Left, Right };
      // static Orientation dir = Up;
      // ImGui::Combo("Direction", (int *)(&dir), "Up\0Down\0Left\0Right\0\0");
//
      // // We can also use a std::vector<std::string> defined dynamically
      // static int num_choices = 4;
      // static std::vector<std::string> choices;
      // static int idx_choice = 0;
      // if (ImGui::InputInt("Num letters", &num_choices)) {
        // num_choices = std::max(1, std::min(26, num_choices));
      // }
//
      // if (num_choices != (int)choices.size()) {
        // choices.resize(num_choices);
        // for (int i = 0; i < num_choices; ++i)
          // choices[i] = std::string(1, 'A' + i);
        // if (idx_choice >= num_choices)
          // idx_choice = num_choices - 1;
      // }
      // ImGui::Combo("Letter", &idx_choice, choices);
//
      // // Add a button
      // if (ImGui::Button("Print Hello", ImVec2(-1, 0))) {
        // std::cout << "Hello\n";
      // }
    // }
  // };

//}}}
