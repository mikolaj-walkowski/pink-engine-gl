#pragma once

#include "nvvkhl/appbase_vk.hpp"
#include "nvvk/debug_util_vk.hpp"
#include "nvvk/descriptorsets_vk.hpp"
#include "nvvk/memallocator_dma_vk.hpp"
#include "nvvk/resourceallocator_vk.hpp"
#include "host_device.h"
#include "nvvk/context_vk.hpp"
#include "pink_structs.hpp"


//--------------------------------------------------------------------------------------------------
// Simple rasterizer of OBJ objects
// - Each OBJ loaded are stored in an `ObjModel` and referenced by a `ObjInstance`
// - It is possible to have many `ObjInstance` referencing the same `ObjModel`
// - Rendering is done in an offscreen framebuffer
// - The image of the framebuffer is displayed in post-process in a full-screen quad
//
class SolidColor: public nvvkhl::AppBaseVk
{
public:
    nvmath::vec4f clearColor;
    ps::WordState* w1;
    ps::WordState* w2;
    float dT;
    SolidColor(nvvk::Context* vkctx, GLFWwindow* window, const int w, const int h);
    void setup(const VkInstance& instance, const VkDevice& device, const VkPhysicalDevice& physicalDevice, uint32_t queueFamily) override;
    void createDescriptorSetLayout();
    void createGraphicsPipeline();
    void updateDescriptorSet();
    void createUniformBuffer();
    void createTextureImages(const VkCommandBuffer& cmdBuf, const std::vector<std::string>& textures);
    void updateUniformBuffer(const VkCommandBuffer& cmdBuf);
    void onResize(int /*w*/, int /*h*/) override;
    void destroyResources();
    void rasterize(const VkCommandBuffer& cmdBuff);
    void rasterizeHelper(const VkCommandBuffer&, std::pair<std::vector<ps::Object>*, std::vector<ps::Object>*>);

    void drawFrame(ps::WordState*, ps::WordState*,float);
    void renderUI();
    // Information pushed at each draw call
    PushConstantRaster m_pcRaster{
        {1},                // Identity matrix
        {10.f, 15.f, 8.f},  // light position
        0,                  // instance Id
        100.f,              // light intensity
        0                   // light type
    };

    // Graphic pipeline
    VkPipelineLayout            m_pipelineLayout;
    VkPipeline                  m_graphicsPipeline;
    nvvk::DescriptorSetBindings m_descSetLayoutBind;
    VkDescriptorPool            m_descPool;
    VkDescriptorSetLayout       m_descSetLayout;
    VkDescriptorSet             m_descSet;

    nvvk::Buffer m_bGlobals;  // Device-Host of the camera matrices
    nvvk::Buffer m_bObjDesc;  // Device buffer of the OBJ descriptions

    std::vector<nvvk::Texture> m_textures;  // vector of all textures of the scene


    nvvk::ResourceAllocatorDma m_alloc;  // Allocator for buffer, images, acceleration structures
    nvvk::DebugUtil            m_debug;  // Utility to name objects


    // #Post - Draw the rendered image on a quad using a tonemapper
    void createOffscreenRender();
    void createPostPipeline();
    void createPostDescriptor();
    void updatePostDescriptorSet();
    void drawPost(VkCommandBuffer cmdBuf);

    nvvk::DescriptorSetBindings m_postDescSetLayoutBind;
    VkDescriptorPool            m_postDescPool{ VK_NULL_HANDLE };
    VkDescriptorSetLayout       m_postDescSetLayout{ VK_NULL_HANDLE };
    VkDescriptorSet             m_postDescSet{ VK_NULL_HANDLE };
    VkPipeline                  m_postPipeline{ VK_NULL_HANDLE };
    VkPipelineLayout            m_postPipelineLayout{ VK_NULL_HANDLE };
    VkRenderPass                m_offscreenRenderPass{ VK_NULL_HANDLE };
    VkFramebuffer               m_offscreenFramebuffer{ VK_NULL_HANDLE };
    nvvk::Texture               m_offscreenColor;
    nvvk::Texture               m_offscreenDepth;
    VkFormat                    m_offscreenColorFormat{ VK_FORMAT_R32G32B32A32_SFLOAT };
    VkFormat                    m_offscreenDepthFormat{ VK_FORMAT_X8_D24_UNORM_PACK32 };
};