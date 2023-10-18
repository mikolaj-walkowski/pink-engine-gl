#include "ui.hpp"

#include "backends/imgui_impl_glfw.h"

UI::UI() {}

void UI::show() {
    if (ImGui::BeginMainMenuBar()) {
        ImGui::MenuItem("General", "", &showGeneral);
        ImGui::MenuItem("Graphics", "", &showGraphics);
        ImGui::MenuItem("Physics", "", &showPhysics);
#ifndef NDEBUG
        ImGui::PushItemWidth(150);
        ImGui::DragFloat("##", &pe->interpolation_props.step, 0.02f, 0.0000000001f, 1.0f, "Engine Speed: %.4f", 0);
        if (ImGui::Button(pe->debug_data.stop ? "Start" : "Stop")) {
            pe->debug_data.stop = !pe->debug_data.stop;
            pe->debug_data.stepped = false;
        }
        ImGui::Checkbox("Stepped", &pe->debug_data.stepped);
        if (ImGui::Button("Step")) {
            pe->debug_data.step = 1;
        }

#endif
        ImGui::EndMainMenuBar();
    }
    PhysicsWindow();
    GraphicsWindow();
    GeneralWindow();
}

void UI::init(GLFWwindow* w, bool b, ps::pp::Engine* _p, SolidColor* _g) {
    pe = _p;
    ge = _g;
    ImGui_ImplGlfw_InitForVulkan(w, b);
}

void klnPointToTable(kln::point& p) {
    ImGui::TableNextRow();

    ImGui::TableNextColumn();
    ImGui::Text("%f", p.e013());

    ImGui::TableNextColumn();
    ImGui::Text("%f", p.e021());

    ImGui::TableNextColumn();
    ImGui::Text("%f", p.e032());

    ImGui::TableNextColumn();
    ImGui::Text("%f", p.e123());
}

void klnPointTableHeader() {
    ImGui::TableNextColumn();
    ImGui::Text("e013");

    ImGui::TableNextColumn();
    ImGui::Text("e021");

    ImGui::TableNextColumn();
    ImGui::Text("e032");

    ImGui::TableNextColumn();
    ImGui::Text("e123");
}

void klnLineTableHeader() {
    ImGui::TableNextColumn();
    ImGui::Text("e23");

    ImGui::TableNextColumn();
    ImGui::Text("e31");

    ImGui::TableNextColumn();
    ImGui::Text("e12");

    ImGui::TableNextColumn();
    ImGui::Text("e01");

    ImGui::TableNextColumn();
    ImGui::Text("e02");

    ImGui::TableNextColumn();
    ImGui::Text("e03");
}

void klnLine(std::string name, kln::line l) {
    ImGui::Text(name.c_str());

    if (ImGui::BeginTable(name.c_str(), 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        klnLineTableHeader();
        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::Text("%f", l.e23());
        ImGui::TableNextColumn();
        ImGui::Text("%f", l.e31());
        ImGui::TableNextColumn();
        ImGui::Text("%f", l.e12());
        ImGui::TableNextColumn();
        ImGui::Text("%f", l.e01());
        ImGui::TableNextColumn();
        ImGui::Text("%f", l.e02());
        ImGui::TableNextColumn();
        ImGui::Text("%f", l.e03());
        ImGui::EndTable();
    }
}

void UI::PhysicsWindow() {
    if (showPhysics) {
        ImGui::SetNextWindowPos(ImVec2(20, 30), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(450, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Physics Debug", NULL);
#ifndef NDEBUG
        // ImGui::BeginGroup();
        // {
        //     ImGui::PushItemWidth(100);
        //     ImGui::DragFloat("##", &pe->interpolation_props.step, 0.02f, 0.0000000001f, 1.0f, "Engine Speed: %.4f", 0);
        //     ImGui::SameLine();

        //     if (ImGui::Button(pe->debug_data.stop ? "Start" : "Stop", { 100,30 })) {
        //         pe->debug_data.stop = !pe->debug_data.stop;
        //     }
        //     ImGui::SameLine();
        //     ImGui::Checkbox("Stepped", &pe->debug_data.stepped);
        //     ImGui::SameLine();

        //     if (ImGui::Button("Step", { 100,30 })) {
        //         pe->debug_data.step = 1;
        //     }
        //     ImGui::EndGroup();
        // }
        // Add Obj 
        // Object Starting Coords

        // Object Starting Vel 

        //Object Type
            //Object props

        //Collisions

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNode("Collisions")) {
            for (int n = 0; n < pe->debug_data.collisions.size(); n++)
            {
                auto label = pe->debug_data.collisions[n];
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (ImGui::TreeNode(label.c_str())) {
                    auto m = pe->debug_data.collisionData[n];
                    auto nl = m.normal;

                    //klnLine("Normal", nl);
                    if (ImGui::BeginTable("Points of contact", 4))
                    {
                        klnPointTableHeader();
                        for (int c = 0; c < m.count; c++) {
                            auto p = m.pointsOfContact[c];
                            klnPointToTable(p);
                        }
                        ImGui::EndTable();
                    }
                    ImGui::TreePop();
                }
            }
            ImGui::TreePop();
        }
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNode("RigidBody props")) {
            // auto sim = pe->out->simulatedObjects;
            // for (int i = 0; i < sim.size(); i++) {
            //     ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            //     if (ImGui::TreeNode((std::to_string(sim[i].id) + "<" + ps::pp::shapeName[sim[i].rigidbody.shape->type] + ">").c_str())) {
            //         klnLine("dB", sim[i].rigidbody.dB);
            //         klnLine("B", sim[i].rigidbody.B);

            //         klnLine("dM", sim[i].rigidbody.dM);
            //         klnLine("M", sim[i].rigidbody.B);
            //         ImGui::TreePop();
            //     }
            // }
            ImGui::TreePop();
        }
#endif
        ImGui::End();
    }
}

void UI::GraphicsWindow() {
    if (showGraphics) {
        ImGui::SetNextWindowPos(ImVec2(20, 340), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Graphics Debug", NULL);
        ImGui::ColorEdit3("Clear color", reinterpret_cast<float*>(&(ge->clearColor)));
        ge->renderUI();
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGuiH::Control::Info("", "", "(F10) Toggle Pane", ImGuiH::Control::Flags::Disabled);
        ImGui::End();
    }
}

void UI::GeneralWindow() {
    if (showGeneral) 
    {
        // TODO make this pretty, idk how to use ImGui

        ImGui::SetNextWindowPos(ImVec2(480, 30), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("General Debug", NULL);

        if (ImGui::CollapsingHeader("Create new object"))
        {
            static bool dynamic_physics = false;
            static float pos_X = 0, pos_Y = 0, pos_Z = 0;
            static float rot_X = 0, rot_Y = 0, rot_Z = 0;
            static float sca_X = 1, sca_Y = 1, sca_Z = 1;
            static float vel_X = 0, vel_Y = 0, vel_Z = 0;
            static float mass = 1;

            if (ImGui::CollapsingHeader("Object coords"))
            {
                ImGui::InputFloat("X position", &pos_X);
                ImGui::InputFloat("Y position", &pos_Y);
                ImGui::InputFloat("Z position", &pos_Z);
            }

            if (ImGui::CollapsingHeader("Object rotation"))
            {
                ImGui::InputFloat("X rotation", &rot_X);
                ImGui::InputFloat("Y rotation", &rot_Y);
                ImGui::InputFloat("Z rotation", &rot_Z);
            }

            if (ImGui::CollapsingHeader("Object scale"))
            {
                ImGui::InputFloat("X scale", &sca_X);
                ImGui::InputFloat("Y scale", &sca_Y);
                ImGui::InputFloat("Z scale", &sca_Z);
            }

            ImGui::Checkbox("Is dynamic", &dynamic_physics);
            if(dynamic_physics)
            {
                ImGui::InputFloat("X velocity", &vel_X);
                ImGui::InputFloat("Y velocity", &vel_Y);
                ImGui::InputFloat("Z velocity", &vel_Z);
                ImGui::InputFloat("Object mass", &mass);
            }

            // FIXME do this better, this is just the initial version
            // ge->objChoice() or smth here? It would be good to be able to choose any object from ObjLibrary
            // For now I'll just add the primitives
            const char* items[] = { "Box", "Sphere", "Plane", /*"Cylinder"*/};
            static const char* current_item = items[0];

            if (ImGui::BeginCombo("Shape", current_item))
            {
                for (int n = 0; n < IM_ARRAYSIZE(items); n++)
                {
                    bool is_selected = (current_item == items[n]);
                    if (ImGui::Selectable(items[n], is_selected))
                        current_item = items[n];
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            // Oh God, what have I done
            if(ImGui::Button("Create") && current_item != NULL)
            {
                ps::pp::BaseShape* shape;
                if (!strcmp(current_item, "Box"))
                    shape = new ps::pp::Box(sca_X, sca_Y, sca_Z, mass, kln::uMotor());
                else if (!strcmp(current_item, "Cylinder"))
                    shape = new ps::pp::Cylinder(sca_Y, sca_X, mass, kln::uMotor());   
                                // NOTE: The X and Z scale make it a bit more complex; this is not correct
                else if (!strcmp(current_item, "Sphere"))
                    shape = new ps::pp::Sphere(sca_X, mass, kln::uMotor());     
                                // NOTE: This is not correct at all
                else if (!strcmp(current_item, "Plane"))
                {
                    if (dynamic_physics)
                        // You can't do that!!! Probably some better handling should be placed here but I just can't be bothered
                        dynamic_physics = false;

                    shape = new ps::pp::Plane(kln::plane(0, 1, 0, 0));
                }

                ps::Object* createdObj = utils::objectCreate(
                    ps::ObjectManager::GetInstance(),
                    // position
                    kln::translator(pos_X, 1, 0, 0) * 
                    kln::translator(pos_Y, 0, 1, 0) * 
                    kln::translator(pos_Z, 0, 0, 1) * 
                    // rotation (pi/180 = around 0.01745)
                    kln::rotor(rot_X * 0.01745f, 1, 0, 0) *  
                    kln::rotor(rot_Y * 0.01745f, 0, 1, 0) * 
                    kln::rotor(rot_Z * 0.01745f, 0, 0, 1)
                    ,
                    dynamic_physics? ps::pp::BT_DYNAMIC : ps::pp::BT_STATIC,
                    "",
                    shape,
                    nvmath::scale_mat4(nvmath::vec3f(sca_X, sca_Y, sca_Z)),
                    kln::line(vel_X, vel_Y, vel_Z, 0, 0, 0)
                );
            };
        }
        ImGui::End();
    }
}

void UI::newFrame() {
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}