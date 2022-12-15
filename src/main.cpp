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
#define WC_SIZE 4
//////////////////////////////////////////////////////////////////////////

int euclidean_remainder(int a, int b)
{
  assert(b != 0);
  int r = a % b;
  return r >= 0 ? r : r + std::abs(b);
}

ps::WordState wordChain[WC_SIZE]; // Buffer dla kolejnych stanów 


//--------------------------------------------------------------------------------------------------
// Application Entry
//
int main(int argc, char** argv)
{
  UNUSED(argc);
  

  SolidColor graphicsEngine;
  ps::pp::Engine physicsEngine(ps::pp::basicSimulate, ps::pp::basicCollider, ps::pp::basicResolver, ps::pp::eulerInterpolation);

  GLFWwindow* window = utils::glfw::setupGLFWindow();
  // setup some basic things for the sample, logging file for example
  NVPSystem system(PROJECT_NAME);

  // Creating Vulkan base application
  nvvk::Context vkctx{};

  utils::nvidia::setupContext(&vkctx, { utils::glfw::getGLFWExtensions() });



  // Create example
  graphicsEngine.init(&vkctx, window, utils::glfw::SAMPLE_WIDTH, utils::glfw::SAMPLE_HEIGHT);

  // ps::Object object;
  // object.mesh = objLib.GetMesh("cube_multi");
  // object.rigidbody.M = kln::translator(1, 1, 0, 0);'
  ps::pp::Sphere o1 = { 1.f,kln::point(0,0,0) };
  ps::Object object1 = utils::objectCreate(kln::motor(kln::translator(6, -1, 1, 0)), ps::pp::ST_SPHERE, &o1);

  ps::pp::Sphere o2 = { 1.f,kln::point(0,0,0) };
  ps::Object object2 = utils::objectCreate(kln::motor(kln::translator(-3, 1, 0, 0)), ps::pp::ST_SPHERE, &o2);


  ps::pp::Plane o3 = { kln::plane(0,1,0,0) };
  ps::Object object3 = utils::objectCreate(kln::motor(kln::translator(-3, 0, 1, 0)), ps::pp::ST_PLANE, &o3);


  wordChain[0].simulatedObjects.push_back(object1);
  wordChain[0].simulatedObjects.push_back(object2);
  wordChain[0].staticObjects.push_back(object3);


  graphicsEngine.setupGlfwCallbacks(window);
  ImGui_ImplGlfw_InitForVulkan(window, true);
  int now = 0;
  int next = 1;
  
  physicsEngine.step(&wordChain[now], &wordChain[next]);

  static double limitFPS = 1.0 / 60.0;

  double lastTime = glfwGetTime(), timer = lastTime;
  double deltaTime = 0, nowTime = 0;

  // Main loop
  while (!glfwWindowShouldClose(window))
  {
    
    glfwPollEvents();
    if (graphicsEngine.isMinimized())
      continue;


    // TODO Zajebane z neta 
    // - Measure time
    nowTime = glfwGetTime();
    deltaTime += (nowTime - lastTime) / limitFPS;
    lastTime = nowTime;

    // - Only update at 60 frames / s
    while (deltaTime >= 1.0) {
      next = (now + 1) % WC_SIZE;
      physicsEngine.step(&wordChain[now], &wordChain[next]);
      deltaTime--;
      now = next;
    }

    // DO WYMIANY 
    
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

    nowTime = glfwGetTime();
    graphicsEngine.drawFrame(&wordChain[euclidean_remainder(now-1,WC_SIZE)], &wordChain[now], (float)deltaTime);

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