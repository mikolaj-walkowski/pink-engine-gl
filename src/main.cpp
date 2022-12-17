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
#include "ui.hpp"

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

  //Setup Glfw
  GLFWwindow* window = utils::glfw::setupGLFWindow();

  // Setup some basic things for the sample, logging file for example
  NVPSystem system(PROJECT_NAME);

  // Creating Vulkan base application
  nvvk::Context vkctx{};

  // Setup device
  utils::nvidia::setupContext(&vkctx, { utils::glfw::getGLFWExtensions() });

  // Setup graphics engine + vulkan memory allocator + nvidia debug
  SolidColor graphicsEngine(&vkctx, window, utils::glfw::SAMPLE_WIDTH, utils::glfw::SAMPLE_HEIGHT);

  //Setup physics engine components
  ps::pp::Engine physicsEngine(ps::pp::basicSimulate, ps::pp::basicCollider, ps::pp::basicResolver, ps::pp::eulerInterpolation);

  /// DEBUG ZONE ========
  // CREATING objects
  ps::Object object1 = utils::objectCreate(
    kln::translator(-9, 1, 0, 0) * kln::rotor(kln::pi_4, 1, 1, 1),
    ps::pp::ST_BOX,
    &utils::boxCreate(1, 1, 1, 1, kln::uMotor())
  );

  ps::Object object2 = utils::objectCreate(
    kln::translator(-3, 1, 0, 0),
    ps::pp::ST_SPHERE,
    &utils::sphereCreate(1.f, 1.f, kln::uMotor())
  );

  ps::Object object3 = utils::objectCreate(
    kln::translator(-3, 0, 1, 0),
    ps::pp::ST_PLANE,
    &utils::planeCreate(kln::plane(0, 1, 0, 0))
  );


  wordChain[0].simulatedObjects.push_back(object1);
  wordChain[0].simulatedObjects.push_back(object2);
  wordChain[0].staticObjects.push_back(object3);

  ///  DEBUG ZONE ======== 

  // Init imgui
  UI::D().init(window, true, &physicsEngine, &graphicsEngine);
  int now = 0;
  int next = 1;


  static float limitFPS = 1.0f / 15.0f;

  static float dT = 1000 * limitFPS;

  physicsEngine.step(&wordChain[now], &wordChain[next], dT);

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

    // - Only update at 60 frames / s
    if (deltaTime >= 1.0) {
      next = (now + 1) % WC_SIZE;
      physicsEngine.step(&wordChain[now], &wordChain[next], (float)(nowTime - lastTime));
      lastTime = nowTime;
      now = next;
    }

    // DO WYMIANY 

    // Start the Dear ImGui frame
    UI::D().newFrame();

    // Show UI window.
    if (graphicsEngine.showGui())
    {
      UI::D().show();
      //ImGui::ShowDemoWindow(NULL);
    }

    nowTime = glfwGetTime();
    graphicsEngine.drawFrame(&wordChain[euclidean_remainder(now - 1, WC_SIZE)], &wordChain[now], (float)deltaTime);
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