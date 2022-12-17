#include "nvmath/nvmath.h"
#include "nvvk/descriptorsets_vk.hpp"
#include "nvvk/raytraceKHR_vk.hpp"
#include "nvvk/sbtwrapper_vk.hpp"
#include "obj.hpp"
#include "host_device.h"

class Raytracer
{
public:
  void setup(const VkDevice& device, const VkPhysicalDevice& physicalDevice, nvvk::ResourceAllocator* allocator, uint32_t queueFamily);
  void destroy();

  auto objectToVkGeometryKHR(const ObjModel& model);
  auto implicitToVkGeometryKHR(const ImplInst& implicitObj);
  void createBottomLevelAS(std::vector<ObjModel>& models, ImplInst& implicitObj);
  void createTopLevelAS(std::vector<ObjInstance>& instances, ImplInst& implicitObj);
  void createRtDescriptorSet(const VkImageView& outputImage);
  void updateRtDescriptorSet(const VkImageView& outputImage);
  void createRtPipeline(VkDescriptorSetLayout& sceneDescLayout);
  void raytrace(const VkCommandBuffer& cmdBuf,
                const nvmath::vec4f&   clearColor,
                VkDescriptorSet&       sceneDescSet,
                VkExtent2D&            size,
                PushConstantRaster&    sceneConstants);

private:
  std::vector<std::string> m_defaultSearchPaths;
  nvvk::ResourceAllocator* m_alloc{nullptr};  // Allocator for buffer, images, acceleration structures
  VkPhysicalDevice         m_physicalDevice;
  VkDevice                 m_device;
  int                      m_graphicsQueueIndex{0};
  nvvk::DebugUtil          m_debug;  // Utility to name objects
  nvvk::SBTWrapper         m_sbtWrapper;

  VkPhysicalDeviceRayTracingPipelinePropertiesKHR m_rtProperties{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR};
  nvvk::RaytracingBuilderKHR                      m_rtBuilder;
  nvvk::DescriptorSetBindings                     m_rtDescSetLayoutBind;
  VkDescriptorPool                                m_rtDescPool;
  VkDescriptorSetLayout                           m_rtDescSetLayout;
  VkDescriptorSet                                 m_rtDescSet;
  std::vector<VkRayTracingShaderGroupCreateInfoKHR> m_rtShaderGroups;
  VkPipelineLayout                                  m_rtPipelineLayout;
  VkPipeline                                        m_rtPipeline;
  nvvk::Buffer                                      m_rtSBTBuffer;


  PushConstantRay m_pcRay{};
};