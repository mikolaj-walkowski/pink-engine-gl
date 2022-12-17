#pragma once
#include "nvvk/debug_util_vk.hpp"
#include "nvvk/descriptorsets_vk.hpp"
#include "nvvk/resourceallocator_vk.hpp"

//--------------------------------------------------------------------------------------------------
// Class to render in off-screen framebuffers. Instead of rendering directly to the
// screen back buffer, this class create the output frame buffer 'createFramebuffer',
// and use the pipeline from 'createPipeline' to render a quad 'draw' with the colorTexture
// image

class Offscreen
{
public:
  void setup(const VkDevice& device, const VkPhysicalDevice& physicalDevice, nvvk::ResourceAllocator* allocator, uint32_t queueFamily);
  void destroy();

  void createFramebuffer(const VkExtent2D& size);
  void createPipeline(VkRenderPass& renderPass);
  void createDescriptor();
  void updateDescriptorSet();
  void draw(VkCommandBuffer cmdBuf, const VkExtent2D& size);

  const VkRenderPass&  renderPass() { return m_renderPass; }
  const VkFramebuffer& frameBuffer() { return m_framebuffer; }
  const nvvk::Texture& colorTexture() { return m_colorTexture; }

private:
  std::vector<std::string> m_defaultSearchPaths;
  nvvk::DescriptorSetBindings m_dsetLayoutBinding;
  VkDescriptorPool            m_descPool{VK_NULL_HANDLE};
  VkDescriptorSetLayout       m_dsetLayout{VK_NULL_HANDLE};
  VkDescriptorSet             m_dset{VK_NULL_HANDLE};
  VkPipeline                  m_pipeline{VK_NULL_HANDLE};
  VkPipelineLayout            m_pipelineLayout{VK_NULL_HANDLE};
  VkRenderPass                m_renderPass{VK_NULL_HANDLE};
  VkFramebuffer               m_framebuffer{VK_NULL_HANDLE};

  nvvk::Texture m_colorTexture;
  VkFormat      m_colorFormat{VK_FORMAT_R32G32B32A32_SFLOAT};
  nvvk::Texture m_depthTexture;
  VkFormat      m_depthFormat{VK_FORMAT_X8_D24_UNORM_PACK32};

  nvvk::ResourceAllocator* m_alloc{nullptr};  // Allocator for buffer, images, acceleration structures
  VkDevice                 m_device;
  int                      m_graphicsQueueIndex{0};
  nvvk::DebugUtil          m_debug;  // Utility to name objects
};