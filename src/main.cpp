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
#include "algorithm"

#include "pink_physics.hpp"
#include "pink_graphics.hpp"
#include "pink_core.hpp"
#include <thread>

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
  ps::pp::Engine physicsEngine(ps::pp::basicSimulate, ps::pp::vecCollider, ps::pp::basicResolver, ps::pp::verletIntegration);

  ps::ObjectManager objectManager({ &physicsEngine, &graphicsEngine });

  /// DEBUG ZONE ========
  // CREATING objects 

  kln::euler_angles a = { kln::pi / 8.f, 0.f, 0.f };
  kln::euler_angles b = { kln::pi / 4.f, kln::pi / 4.f, 0.f };
  auto test = new ps::pp::Box(1, 1, 1, 2, kln::uMotor());

  ps::Object* boxObj = utils::objectCreate(
    objectManager,
    (kln::motor)kln::translator(3.f, 1.f, 0.f, 0.f), //* kln::rotor(kln::pi_4, 1.f, 0.f, 1.f),
    ps::pp::BT_DYNAMIC,
    "",
    test,//new ps::pp::Box(1, 1, 1, 2, kln::uMotor()),
    nvmath::scale_mat4(nvmath::vec3f(1.f, 1.f, 1.f))
  );

  ps::Object* boxObj2 = utils::objectCreate(
    objectManager,
    ((kln::motor)kln::translator(3.f, 1.f, 0.f, 0.f) * kln::translator(3.0f, 0.0f, 1.0f, 0.0f) * kln::rotor(b)).normalized(),
    ps::pp::BT_DYNAMIC,
    "",
    new ps::pp::Box(1, 1, 1, 2, kln::uMotor()),
    nvmath::scale_mat4(nvmath::vec3f(1.f, 1.f, 1.f))
  );

  ps::Object* sphereObj = utils::objectCreate(
    objectManager,
    kln::translator(-3, 1, 0, 0),
    ps::pp::BT_DYNAMIC,
    "",
    new ps::pp::Sphere(1.f, 2.f, kln::uMotor()),
    nvmath::scale_mat4(nvmath::vec3f(1.f, 1.f, 1.f))
  );

  ps::Object* sphereObj2 = utils::objectCreate(
    objectManager,
    kln::translator(-3, 1, 0, 0) * kln::translator(3, 0.1f, 1, 0.0f),
    ps::pp::BT_DYNAMIC,
    "",
    new ps::pp::Sphere(1.f, 2.f, kln::uMotor()),
    nvmath::scale_mat4(nvmath::vec3f(1.f, 1.f, 1.f))
  );

  ps::Object* planeObj = utils::objectCreate(
    objectManager,
    kln::translator(-3, 0, 1, 0),// * kln::rotor(a),
    ps::pp::BT_STATIC,
    "",
    new ps::pp::Plane(kln::plane(0, 1, 0, 0)),
    nvmath::scale_mat4(nvmath::vec3f(10.f, 10.f, 10.f))
  );

  // utils::createCar(objectManager,&physicsEngine,(kln::motor)kln::translator(3.f, 1.f, 0.f, 0.f));

  // physicsEngine.springs.push_back({ 0, 1, 5.f, -0.1f });
  // physicsEngine.joins.push_back(
  //   { 0,1,3,
  //   {boxObj.rigidbody.moved->center,boxObj2.rigidbody.moved->center},
  //   (kln::line(1,1,1,1,1,1) - (boxObj.rigidbody.moved->center & boxObj2.rigidbody.moved->center)).normalized()
  //   });

  ///  DEBUG ZONE ======== 

  // Init imgui
  UI::D().init(window, true, &physicsEngine, &graphicsEngine);
  int now = 0;
  int next = 1;
  bool kill = false;

  static float limitFPS = 1.0f / 15.0f;

  static float dT = 1000 * limitFPS;

  float lastTime = (float)glfwGetTime(), timer = lastTime;
  float deltaTime = 0, nowTime = 0;

  std::thread physics(&ps::pp::Engine::run, &physicsEngine, &now, &lastTime, &kill);

  // Main loop
  while (!glfwWindowShouldClose(window))
  {
    glfwPollEvents();

    //- Measure time 
    nowTime = (float)glfwGetTime();
    deltaTime += (nowTime - lastTime) / limitFPS;

    if (graphicsEngine.isMinimized())
      continue;

    // Start the Dear ImGui frame
    UI::D().newFrame();

    // Show UI window.
    if (graphicsEngine.showGui())
    {
      UI::D().show();
      //ImGui::ShowDemoWindow(NULL);
    }

    nowTime = (float)glfwGetTime();
    graphicsEngine.drawFrame(euclidean_remainder(now - 1, WC_SIZE), now, std::min(deltaTime, 1.0f));
  }

  kill = true;
  // Cleanup
  vkDeviceWaitIdle(graphicsEngine.getDevice());
  physics.join();

  graphicsEngine.destroyResources();
  graphicsEngine.destroy();
  vkctx.deinit();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}