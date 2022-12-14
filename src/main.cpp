/*
 * Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2014-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */


 // ImGui - standalone example application for Glfw + Vulkan, using programmable
 // pipeline If you are new to ImGui, see examples/README.txt and documentation
 // at the top of imgui.cpp.

#include <array>

#include "backends/imgui_impl_glfw.h"
#include "imgui.h"

#include "utils.hpp"

#include "imgui/imgui_camera_widget.h"
#include "nvh/cameramanipulator.hpp"
#include "nvh/fileoperations.hpp"
#include "nvpsystem.hpp"
#include "nvvk/commands_vk.hpp"
#include "nvvk/context_vk.hpp"

#include "pink_physics.hpp"
#include "pink_graphics.hpp"

//////////////////////////////////////////////////////////////////////////
#define UNUSED(x) (void)(x)
//////////////////////////////////////////////////////////////////////////

// Default search path for shaders
std::vector<std::string> defaultSearchPaths = {
      NVPSystem::exePath() + PROJECT_RELDIRECTORY,
      NVPSystem::exePath() + PROJECT_RELDIRECTORY "..",
      NVPSystem::exePath() + "../",
      std::string(PROJECT_NAME),
};

ps::WordState wordChain[4]; // Buffer dla kolejnych stanów 

//engin
ps::pp::Engine physicsEngine(ps::pp::basicSimulate, ps::pp::basicCollider, ps::pp::basicResolver, ps::pp::eulerInterpolation);


//--------------------------------------------------------------------------------------------------
// Application Entry
//
int main(int argc, char** argv)
{
  UNUSED(argc);

  SolidColor graphicsEngine;

  GLFWwindow* window = utils::glfw::setupGLFWindow();
  // setup some basic things for the sample, logging file for example
  NVPSystem system(PROJECT_NAME);

  // Creating Vulkan base application
  nvvk::Context vkctx{};

  utils::nvidia::setupContext(&vkctx, { utils::glfw::getGLFWExtensions() });

  ObjLibrary objLib;

  // Create example
  graphicsEngine.init(&vkctx, window, &defaultSearchPaths, utils::glfw::SAMPLE_WIDTH, utils::glfw::SAMPLE_HEIGHT, &objLib);

  ps::Object object;
  object.mesh = objLib.GetMesh("cube_multi");
  auto line = point(0,2,0)  & point(0,1,0);
  object.rigidbody.M = pga_I + pga_1 + 1 * e01;

  auto rotor2 = rotor(2 * PI, line);

  ps::Object object2;
  object2.mesh = objLib.GetMesh("sphere");
  object2.rigidbody.M =rotor2* translator(1, line);

  ps::Object object3;
  object3.mesh = objLib.GetMesh("plane");
  object3.rigidbody.M =rotor2* translator(-3, line);

  wordChain[0].simulatedObjects.push_back(object);
  wordChain[0].simulatedObjects.push_back(object2);
  wordChain[0].simulatedObjects.push_back(object3);
  

  graphicsEngine.setupGlfwCallbacks(window);
  ImGui_ImplGlfw_InitForVulkan(window, true);

  // Main loop
  while (!glfwWindowShouldClose(window))
  {
    glfwPollEvents();
    if (graphicsEngine.isMinimized())
      continue;

    // Start the Dear ImGui frame
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Show UI window.
    if (graphicsEngine.showGui())
    {
      ImGuiH::Panel::Begin();
      ImGui::ColorEdit3("Clear color", reinterpret_cast<float*>(&graphicsEngine.clearColor));
      graphicsEngine.renderUI();
      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGuiH::Control::Info("", "", "(F10) Toggle Pane", ImGuiH::Control::Flags::Disabled);
      ImGuiH::Panel::End();
    }

    graphicsEngine.drawFrame(&wordChain[0], &wordChain[1], 1.0f);
  }

  // Cleanup
  vkDeviceWaitIdle(graphicsEngine.getDevice());

  graphicsEngine.destroyResources();
  graphicsEngine.destroy();
  vkctx.deinit();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}