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


#include <sstream>

#define VMA_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#include "pink_graphics.hpp"
#include "stb_image.h"
#include "solid_color.hpp"

#include "backends/imgui_impl_glfw.h"
#include "imgui.h"

#include "nvh/alignment.hpp"
#include "nvh/cameramanipulator.hpp"
#include "nvh/fileoperations.hpp"
#include "nvvk/commands_vk.hpp"
#include "nvvk/descriptorsets_vk.hpp"
#include "nvvk/images_vk.hpp"
#include "nvvk/pipeline_vk.hpp"
#include "nvvk/renderpasses_vk.hpp"
#include "nvvk/shaders_vk.hpp"
#include "nvvk/buffers_vk.hpp"

#include "GLFW/glfw3.h"
#include "nvvk/commands_vk.hpp"
#include "nvvk/context_vk.hpp"

 //extern ps::pg::ObjLibrary m_objLibrary;
  //--------------------------------------------------------------------------------------------------
  // Keep the handle on the device
  // Initialize the tool to do all our allocations: buffers, images
  //
void SolidColor::setup(const VkInstance& instance, const VkDevice& device, const VkPhysicalDevice& physicalDevice, uint32_t queueFamily)
{
    AppBaseVk::setup(instance, device, physicalDevice, queueFamily);
    m_alloc.init(instance, device, physicalDevice);
    m_debug.setup(m_device);
    m_offscreenDepthFormat = nvvk::findDepthFormat(physicalDevice);

    m_offscreen.setup(device, physicalDevice, &m_alloc, queueFamily);
    m_raytrace.setup(device, physicalDevice, &m_alloc, queueFamily);
}

//--------------------------------------------------------------------------------------------------
// Called at each frame to update the camera matrix
//
void SolidColor::updateUniformBuffer(const VkCommandBuffer& cmdBuf)
{
    // Prepare new UBO contents on host.
    const float    aspectRatio = m_size.width / static_cast<float>(m_size.height);
    GlobalUniforms hostUBO = {};
    const auto& view = CameraManip.getMatrix();
    const auto& proj = nvmath::perspectiveVK(CameraManip.getFov(), aspectRatio, 0.1f, 1000.0f);
    // proj[1][1] *= -1;  // Inverting Y for Vulkan (not needed with perspectiveVK).

    hostUBO.viewProj = proj * view;
    hostUBO.viewInverse = nvmath::invert(view);
    hostUBO.projInverse = nvmath::invert(proj);

    // UBO on the device, and what stages access it.
    VkBuffer deviceUBO = m_bGlobals.buffer;
    auto     uboUsageStages = VK_PIPELINE_STAGE_VERTEX_SHADER_BIT;

    // Ensure that the modified UBO is not visible to previous frames.
    VkBufferMemoryBarrier beforeBarrier{ VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER };
    beforeBarrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
    beforeBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    beforeBarrier.buffer = deviceUBO;
    beforeBarrier.offset = 0;
    beforeBarrier.size = sizeof(hostUBO);
    vkCmdPipelineBarrier(cmdBuf, uboUsageStages, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_DEPENDENCY_DEVICE_GROUP_BIT, 0,
        nullptr, 1, &beforeBarrier, 0, nullptr);


    // Schedule the host-to-device upload. (hostUBO is copied into the cmd
    // buffer so it is okay to deallocate when the function returns).
    vkCmdUpdateBuffer(cmdBuf, m_bGlobals.buffer, 0, sizeof(GlobalUniforms), &hostUBO);

    // Making sure the updated UBO will be visible.
    VkBufferMemoryBarrier afterBarrier{ VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER };
    afterBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    afterBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    afterBarrier.buffer = deviceUBO;
    afterBarrier.offset = 0;
    afterBarrier.size = sizeof(hostUBO);
    vkCmdPipelineBarrier(cmdBuf, VK_PIPELINE_STAGE_TRANSFER_BIT, uboUsageStages, VK_DEPENDENCY_DEVICE_GROUP_BIT, 0,
        nullptr, 1, &afterBarrier, 0, nullptr);
}

//--------------------------------------------------------------------------------------------------
// Describing the layout pushed when rendering
//
void SolidColor::createDescriptorSetLayout()
{
    auto nbTxt = static_cast<uint32_t>(ps::pg::ObjLibrary::getObjLibrary().m_textures.size());

    // Camera matrices
    m_descSetLayoutBind.addBinding(SceneBindings::eGlobals, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_RAYGEN_BIT_KHR);
    // Obj descriptions
    m_descSetLayoutBind.addBinding(SceneBindings::eObjDescs, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR);
    // Textures
    m_descSetLayoutBind.addBinding(SceneBindings::eTextures, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, nbTxt,
        VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR);


    m_descSetLayout = m_descSetLayoutBind.createLayout(m_device);
    m_descPool = m_descSetLayoutBind.createPool(m_device, 1);
    m_descSet = nvvk::allocateDescriptorSet(m_device, m_descPool, m_descSetLayout);
}

//--------------------------------------------------------------------------------------------------
// Setting up the buffers in the descriptor set
//
void SolidColor::updateDescriptorSet()
{
    std::vector<VkWriteDescriptorSet> writes;

    // Camera matrices and scene description
    VkDescriptorBufferInfo dbiUnif{ m_bGlobals.buffer, 0, VK_WHOLE_SIZE };
    writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, SceneBindings::eGlobals, &dbiUnif));

    VkDescriptorBufferInfo dbiSceneDesc{ ps::pg::ObjLibrary::getObjLibrary().m_bObjDesc.buffer, 0, VK_WHOLE_SIZE };
    writes.emplace_back(m_descSetLayoutBind.makeWrite(m_descSet, SceneBindings::eObjDescs, &dbiSceneDesc));

    // All texture samplers
    std::vector<VkDescriptorImageInfo> diit;
    for (auto& texture : ps::pg::ObjLibrary::getObjLibrary().m_textures)
    {
        diit.emplace_back(texture.descriptor);
    }
    writes.emplace_back(m_descSetLayoutBind.makeWriteArray(m_descSet, SceneBindings::eTextures, diit.data()));

    // Writing the information
    vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
}


//--------------------------------------------------------------------------------------------------
// Creating the pipeline layout
//
void SolidColor::createGraphicsPipeline()
{
    VkPushConstantRange pushConstantRanges = { VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushConstantRaster) };

    // Creating the Pipeline Layout
    VkPipelineLayoutCreateInfo createInfo{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    createInfo.setLayoutCount = 1;
    createInfo.pSetLayouts = &m_descSetLayout;
    createInfo.pushConstantRangeCount = 1;
    createInfo.pPushConstantRanges = &pushConstantRanges;
    vkCreatePipelineLayout(m_device, &createInfo, nullptr, &m_pipelineLayout);


    // Creating the Pipeline
    std::vector<std::string>                paths = ps::defaultSearchPaths;
    nvvk::GraphicsPipelineGeneratorCombined gpb(m_device, m_pipelineLayout, m_offscreenRenderPass);
    gpb.depthStencilState.depthTestEnable = true;
    gpb.addShader(nvh::loadFile("spv/vert_shader.vert.spv", true, paths, true), VK_SHADER_STAGE_VERTEX_BIT);
    gpb.addShader(nvh::loadFile("spv/frag_shader.frag.spv", true, paths, true), VK_SHADER_STAGE_FRAGMENT_BIT);
    gpb.addBindingDescription({ 0, sizeof(VertexObj) });
    gpb.addAttributeDescriptions({
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<uint32_t>(offsetof(VertexObj, pos))},
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<uint32_t>(offsetof(VertexObj, nrm))},
        {2, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<uint32_t>(offsetof(VertexObj, color))},
        {3, 0, VK_FORMAT_R32G32_SFLOAT, static_cast<uint32_t>(offsetof(VertexObj, texCoord))},
        });

    m_graphicsPipeline = gpb.createPipeline();
    m_debug.setObjectName(m_graphicsPipeline, "Graphics");
}

//--------------------------------------------------------------------------------------------------
// Creating the uniform buffer holding the camera matrices
// - Buffer is host visible
//
void SolidColor::createUniformBuffer()
{
    m_bGlobals = m_alloc.createBuffer(sizeof(GlobalUniforms), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    m_debug.setObjectName(m_bGlobals.buffer, "Globals");
}

//--------------------------------------------------------------------------------------------------
// Creating all textures and samplers
//
void SolidColor::createTextureImages(const VkCommandBuffer& cmdBuf, const std::vector<std::string>& textures)
{
    VkSamplerCreateInfo samplerCreateInfo{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    samplerCreateInfo.minFilter = VK_FILTER_LINEAR;
    samplerCreateInfo.magFilter = VK_FILTER_LINEAR;
    samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerCreateInfo.maxLod = FLT_MAX;

    VkFormat format = VK_FORMAT_R8G8B8A8_SRGB;

    // If no textures are present, create a dummy one to accommodate the pipeline layout
    if (textures.empty() && ps::pg::ObjLibrary::getObjLibrary().m_textures.empty())
    {
        nvvk::Texture texture;

        std::array<uint8_t, 4> color{ 255u, 255u, 255u, 255u };
        VkDeviceSize           bufferSize = sizeof(color);
        auto                   imgSize = VkExtent2D{ 1, 1 };
        auto                   imageCreateInfo = nvvk::makeImage2DCreateInfo(imgSize, format);

        // Creating the dummy texture
        nvvk::Image           image = m_alloc.createImage(cmdBuf, bufferSize, color.data(), imageCreateInfo);
        VkImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, imageCreateInfo);
        texture = m_alloc.createTexture(image, ivInfo, samplerCreateInfo);

        // The image format must be in VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        nvvk::cmdBarrierImageLayout(cmdBuf, texture.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        ps::pg::ObjLibrary::getObjLibrary().m_textures.push_back(texture);
    }
    else
    {
        // Uploading all images
        for (const auto& texture : textures)
        {
            std::stringstream o;
            int               texWidth, texHeight, texChannels;
            o << "media/textures/" << texture;
            std::string txtFile = nvh::findFile(o.str(), ps::defaultSearchPaths, true);

            stbi_uc* stbi_pixels = stbi_load(txtFile.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

            std::array<stbi_uc, 4> color{ 255u, 0u, 255u, 255u };

            stbi_uc* pixels = stbi_pixels;
            // Handle failure
            if (!stbi_pixels)
            {
                texWidth = texHeight = 1;
                texChannels = 4;
                pixels = reinterpret_cast<stbi_uc*>(color.data());
            }

            VkDeviceSize bufferSize = static_cast<uint64_t>(texWidth) * texHeight * sizeof(uint8_t) * 4;
            auto         imgSize = VkExtent2D{ (uint32_t)texWidth, (uint32_t)texHeight };
            auto         imageCreateInfo = nvvk::makeImage2DCreateInfo(imgSize, format, VK_IMAGE_USAGE_SAMPLED_BIT, true);

            {
                nvvk::Image image = m_alloc.createImage(cmdBuf, bufferSize, pixels, imageCreateInfo);
                nvvk::cmdGenerateMipmaps(cmdBuf, image.image, format, imgSize, imageCreateInfo.mipLevels);
                VkImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, imageCreateInfo);
                nvvk::Texture         texture = m_alloc.createTexture(image, ivInfo, samplerCreateInfo);

                ps::pg::ObjLibrary::getObjLibrary().m_textures.push_back(texture);
            }

            stbi_image_free(stbi_pixels);
        }
    }
}

//--------------------------------------------------------------------------------------------------
// Destroying all allocations
//
void SolidColor::destroyResources()
{
    vkDestroyPipeline(m_device, m_graphicsPipeline, nullptr);
    vkDestroyPipelineLayout(m_device, m_pipelineLayout, nullptr);
    vkDestroyDescriptorPool(m_device, m_descPool, nullptr);
    vkDestroyDescriptorSetLayout(m_device, m_descSetLayout, nullptr);

    m_alloc.destroy(m_bGlobals);
    m_alloc.destroy(m_bObjDesc);
    m_alloc.destroy(m_implObjects.implBuf);
    m_alloc.destroy(m_implObjects.implMatBuf);
    m_alloc.destroy(ps::pg::ObjLibrary::getObjLibrary().m_bObjDesc);

    for (auto& m : ps::pg::ObjLibrary::getObjLibrary().m_meshContainer)
    {
        m_alloc.destroy(m.vertexBuffer);
        m_alloc.destroy(m.indexBuffer);
        m_alloc.destroy(m.matColorBuffer);
        m_alloc.destroy(m.matIndexBuffer);
    }

    for (auto& t : ps::pg::ObjLibrary::getObjLibrary().m_textures)
    {
        m_alloc.destroy(t);
    }

    //#Post
    m_alloc.destroy(m_offscreenColor);
    m_alloc.destroy(m_offscreenDepth);
    ps::pg::ObjLibrary::getObjLibrary().m_cmdPool.deinit();
    vkDestroyPipeline(m_device, m_postPipeline, nullptr);
    vkDestroyPipelineLayout(m_device, m_postPipelineLayout, nullptr);
    vkDestroyDescriptorPool(m_device, m_postDescPool, nullptr);
    vkDestroyDescriptorSetLayout(m_device, m_postDescSetLayout, nullptr);
    vkDestroyRenderPass(m_device, m_offscreenRenderPass, nullptr);
    vkDestroyFramebuffer(m_device, m_offscreenFramebuffer, nullptr);

    m_alloc.deinit();
}

//--------------------------------------------------------------------------------------------------
// Drawing the scene in raster mode
//
void SolidColor::rasterize(int prev, int now, const VkCommandBuffer& cmdBuf)
{
    m_debug.beginLabel(cmdBuf, "Rasterize");

    // Dynamic Viewport
    setViewport(cmdBuf);

    // Drawing all triangles
    vkCmdBindPipeline(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, m_graphicsPipeline);
    vkCmdBindDescriptorSets(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayout, 0, 1, &m_descSet, 0, nullptr);


    VkDeviceSize offset{ 0 };

    for (int i = 0; i < objects.size(); i++)
    {
        auto model = objects[i]->model.mesh;
        m_pcRaster.objIndex = model->objIndex;  // Telling which object is drawn
        m_pcRaster.modelMatrix = objects[i]->interpolate(prev, now, dT);

        vkCmdPushConstants(cmdBuf, m_pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
            sizeof(PushConstantRaster), &m_pcRaster);
        vkCmdBindVertexBuffers(cmdBuf, 0, 1, &model->vertexBuffer.buffer, &offset);
        vkCmdBindIndexBuffer(cmdBuf, model->indexBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmdBuf, model->nbIndices, 1, 0, 0, 0);
    }

#ifndef NDEBUG
    // VkDeviceSize offset{ 0 };

    // for (int i = 0; i < w2->points.size(); i++)
    // {
    //     auto model = ps::pg::ObjLibrary::getObjLibrary().GetMesh("sphere");
    //     m_pcRaster.objIndex = model->objIndex;  // Telling which object is drawn
    //     m_pcRaster.modelMatrix = w2->points[i];

    //     vkCmdPushConstants(cmdBuf, m_pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
    //         sizeof(PushConstantRaster), &m_pcRaster);
    //     vkCmdBindVertexBuffers(cmdBuf, 0, 1, &model->vertexBuffer.buffer, &offset);
    //     vkCmdBindIndexBuffer(cmdBuf, model->indexBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
    //     vkCmdDrawIndexed(cmdBuf, model->nbIndices, 1, 0, 0, 0);
    // }
#endif
    m_debug.endLabel(cmdBuf);
}

//--------------------------------------------------------------------------------------------------
// Handling resize of the window
//
void SolidColor::onResize(int /*w*/, int /*h*/)
{
    createOffscreenRender();
    updatePostDescriptorSet();
}


//////////////////////////////////////////////////////////////////////////
// Post-processing
//////////////////////////////////////////////////////////////////////////


//--------------------------------------------------------------------------------------------------
// Creating an offscreen frame buffer and the associated render pass
//
void SolidColor::createOffscreenRender()
{
    m_alloc.destroy(m_offscreenColor);
    m_alloc.destroy(m_offscreenDepth);

    // Creating the color image
    {
        auto colorCreateInfo = nvvk::makeImage2DCreateInfo(m_size, m_offscreenColorFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
            | VK_IMAGE_USAGE_STORAGE_BIT);


        nvvk::Image           image = m_alloc.createImage(colorCreateInfo);
        VkImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, colorCreateInfo);
        VkSamplerCreateInfo   sampler{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        m_offscreenColor = m_alloc.createTexture(image, ivInfo, sampler);
        m_offscreenColor.descriptor.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    }

    // Creating the depth buffer
    auto depthCreateInfo = nvvk::makeImage2DCreateInfo(m_size, m_offscreenDepthFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT);
    {
        nvvk::Image image = m_alloc.createImage(depthCreateInfo);


        VkImageViewCreateInfo depthStencilView{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        depthStencilView.viewType = VK_IMAGE_VIEW_TYPE_2D;
        depthStencilView.format = m_offscreenDepthFormat;
        depthStencilView.subresourceRange = { VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1 };
        depthStencilView.image = image.image;

        m_offscreenDepth = m_alloc.createTexture(image, depthStencilView);
    }

    // Setting the image layout for both color and depth
    {
        nvvk::CommandPool genCmdBuf(m_device, m_graphicsQueueIndex);
        auto              cmdBuf = genCmdBuf.createCommandBuffer();
        nvvk::cmdBarrierImageLayout(cmdBuf, m_offscreenColor.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);
        nvvk::cmdBarrierImageLayout(cmdBuf, m_offscreenDepth.image, VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL, VK_IMAGE_ASPECT_DEPTH_BIT);

        genCmdBuf.submitAndWait(cmdBuf);
    }

    // Creating a renderpass for the offscreen
    if (!m_offscreenRenderPass)
    {
        m_offscreenRenderPass = nvvk::createRenderPass(m_device, { m_offscreenColorFormat }, m_offscreenDepthFormat, 1, true,
            true, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL);
    }


    // Creating the frame buffer for offscreen
    std::vector<VkImageView> attachments = { m_offscreenColor.descriptor.imageView, m_offscreenDepth.descriptor.imageView };

    vkDestroyFramebuffer(m_device, m_offscreenFramebuffer, nullptr);
    VkFramebufferCreateInfo info{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
    info.renderPass = m_offscreenRenderPass;
    info.attachmentCount = 2;
    info.pAttachments = attachments.data();
    info.width = m_size.width;
    info.height = m_size.height;
    info.layers = 1;
    vkCreateFramebuffer(m_device, &info, nullptr, &m_offscreenFramebuffer);
}

//--------------------------------------------------------------------------------------------------
// The pipeline is how things are rendered, which shaders, type of primitives, depth test and more
//
void SolidColor::createPostPipeline()
{
    // Push constants in the fragment shader
    VkPushConstantRange pushConstantRanges = { VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(float) };

    // Creating the pipeline layout
    VkPipelineLayoutCreateInfo createInfo{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    createInfo.setLayoutCount = 1;
    createInfo.pSetLayouts = &m_postDescSetLayout;
    createInfo.pushConstantRangeCount = 1;
    createInfo.pPushConstantRanges = &pushConstantRanges;
    vkCreatePipelineLayout(m_device, &createInfo, nullptr, &m_postPipelineLayout);


    // Pipeline: completely generic, no vertices
    nvvk::GraphicsPipelineGeneratorCombined pipelineGenerator(m_device, m_postPipelineLayout, m_renderPass);
    pipelineGenerator.addShader(nvh::loadFile("spv/passthrough.vert.spv", true, ps::defaultSearchPaths, true), VK_SHADER_STAGE_VERTEX_BIT);
    pipelineGenerator.addShader(nvh::loadFile("spv/post.frag.spv", true, ps::defaultSearchPaths, true), VK_SHADER_STAGE_FRAGMENT_BIT);
    pipelineGenerator.rasterizationState.cullMode = VK_CULL_MODE_NONE;
    m_postPipeline = pipelineGenerator.createPipeline();
    m_debug.setObjectName(m_postPipeline, "post");
}

//--------------------------------------------------------------------------------------------------
// The descriptor layout is the description of the data that is passed to the vertex or the
// fragment program.
//
void SolidColor::createPostDescriptor()
{
    m_postDescSetLayoutBind.addBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT);
    m_postDescSetLayout = m_postDescSetLayoutBind.createLayout(m_device);
    m_postDescPool = m_postDescSetLayoutBind.createPool(m_device);
    m_postDescSet = nvvk::allocateDescriptorSet(m_device, m_postDescPool, m_postDescSetLayout);
}


//--------------------------------------------------------------------------------------------------
// Update the output
//
void SolidColor::updatePostDescriptorSet()
{
    VkWriteDescriptorSet writeDescriptorSets = m_postDescSetLayoutBind.makeWrite(m_postDescSet, 0, &m_offscreenColor.descriptor);
    vkUpdateDescriptorSets(m_device, 1, &writeDescriptorSets, 0, nullptr);
}

//--------------------------------------------------------------------------------------------------
// Draw a full screen quad with the attached image
//
void SolidColor::drawPost(VkCommandBuffer cmdBuf)
{
    m_debug.beginLabel(cmdBuf, "Post");

    setViewport(cmdBuf);

    auto aspectRatio = static_cast<float>(m_size.width) / static_cast<float>(m_size.height);
    vkCmdPushConstants(cmdBuf, m_postPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(float), &aspectRatio);
    vkCmdBindPipeline(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, m_postPipeline);
    vkCmdBindDescriptorSets(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, m_postPipelineLayout, 0, 1, &m_postDescSet, 0, nullptr);
    vkCmdDraw(cmdBuf, 3, 1, 0, 0);


    m_debug.endLabel(cmdBuf);
}

SolidColor::SolidColor(nvvk::Context* vkctx, GLFWwindow* window, const int w, const int h) {
    // Window need to be opened to get the surface on which to draw
    const VkSurfaceKHR surface = getVkSurface(vkctx->m_instance, window);

    vkctx->setGCTQueueWithPresent(surface);
    setup(vkctx->m_instance, vkctx->m_device, vkctx->m_physicalDevice, vkctx->m_queueGCT.familyIndex);
    createSwapchain(surface, w, h);
    createDepthBuffer();
    createRenderPass();
    createFrameBuffers();

    // Setup Imgui
    initGUI(0);  // Using sub-pass 0

    // Setup ObjLibrary
    ps::pg::ObjLibrary::getObjLibrary().init(&m_alloc, m_device, m_graphicsQueueIndex, m_debug);

    // Creation of the example
    ps::pg::ObjLibrary::getObjLibrary().LoadDirectory("../../media/scenes/");

    createOffscreenRender();
    createDescriptorSetLayout();
    createGraphicsPipeline();
    createUniformBuffer();
    ps::pg::ObjLibrary::getObjLibrary().CreateObjDescriptionBuffer();
    updateDescriptorSet();

    createPostDescriptor();
    createPostPipeline();
    updatePostDescriptorSet();
    clearColor = nvmath::vec4f(1, 1, 1, 1.00f);

    setupGlfwCallbacks(window);
}

void SolidColor::registerObject(ps::Object* o) {
    objects.insert(std::upper_bound(objects.begin(), objects.end(), o, ps::ObjectIDCmp()), o);
}
void SolidColor::deregisterObject(ps::Object* o) {
    objects.erase(std::upper_bound(objects.begin(), objects.end(), o, ps::ObjectIDCmp()));
}

void SolidColor::drawFrame(int prev, int now, float _dT) {
    dT = _dT;
    // Start rendering the scene
    prepareFrame();

    // Start command buffer of this frame
    auto                   curFrame = getCurFrame();
    const VkCommandBuffer& cmdBuf = getCommandBuffers()[curFrame];

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmdBuf, &beginInfo);

    // Updating camera buffer
    updateUniformBuffer(cmdBuf);

    // Clearing screen
    std::array<VkClearValue, 2> clearValues{};
    clearValues[0].color = { clearColor[0],clearColor[1],clearColor[2],clearColor[3] };
    clearValues[1].depthStencil = { 1.0f, 0 };

    // Offscreen render pass
    {
        VkRenderPassBeginInfo offscreenRenderPassBeginInfo{ VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
        offscreenRenderPassBeginInfo.clearValueCount = 2;
        offscreenRenderPassBeginInfo.pClearValues = clearValues.data();
        offscreenRenderPassBeginInfo.renderPass = m_offscreenRenderPass;
        offscreenRenderPassBeginInfo.framebuffer = m_offscreenFramebuffer;
        offscreenRenderPassBeginInfo.renderArea = { {0, 0}, getSize() };

        // Rendering Scene
        vkCmdBeginRenderPass(cmdBuf, &offscreenRenderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
        rasterize(prev, now, cmdBuf);
        vkCmdEndRenderPass(cmdBuf);
    }


    // 2nd rendering pass: tone mapper, UI
    {
        VkRenderPassBeginInfo postRenderPassBeginInfo{ VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
        postRenderPassBeginInfo.clearValueCount = 2;
        postRenderPassBeginInfo.pClearValues = clearValues.data();
        postRenderPassBeginInfo.renderPass = getRenderPass();
        postRenderPassBeginInfo.framebuffer = getFramebuffers()[curFrame];
        postRenderPassBeginInfo.renderArea = { {0, 0}, getSize() };

        // Rendering tonemapper
        vkCmdBeginRenderPass(cmdBuf, &postRenderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
        drawPost(cmdBuf);
        // Rendering UI
        ImGui::Render();
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmdBuf);
        vkCmdEndRenderPass(cmdBuf);
    }


    // Submit for display
    vkEndCommandBuffer(cmdBuf);
    submitFrame();

}

void SolidColor::renderUI()
{
    ImGuiH::CameraWidget();
    if (ImGui::CollapsingHeader("Light"))
    {
        ImGui::RadioButton("Point", &m_pcRaster.lightType, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Infinite", &m_pcRaster.lightType, 1);

        ImGui::SliderFloat3("Position", &m_pcRaster.lightPosition.x, -20.f, 20.f);
        ImGui::SliderFloat("Intensity", &m_pcRaster.lightIntensity, 0.f, 150.f);
    }
}

//--------------------------------------------------------------------------------------------------
// Initialize offscreen rendering
//
void SolidColor::initOffscreen()
{
    m_offscreen.createFramebuffer(m_size);
    m_offscreen.createDescriptor();
    m_offscreen.createPipeline(m_renderPass);
    m_offscreen.updateDescriptorSet();
}

//--------------------------------------------------------------------------------------------------
// Initialize Vulkan ray tracing
//
void SolidColor::initRayTracing()
{
    //   m_raytrace.createBottomLevelAS(m_objModel, m_implObjects);
    //   m_raytrace.createTopLevelAS(m_instances, m_implObjects);
    m_raytrace.createRtDescriptorSet(m_offscreen.colorTexture().descriptor.imageView);
    m_raytrace.createRtPipeline(m_descSetLayout);
}

//--------------------------------------------------------------------------------------------------
// Ray trace the scene
//
void SolidColor::raytrace(const VkCommandBuffer& cmdBuf, const nvmath::vec4f& clearColor)
{

    m_raytrace.raytrace(cmdBuf, clearColor, m_descSet, m_size, m_pcRaster);
}