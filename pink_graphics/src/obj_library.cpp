#include "pink_structs.hpp"
#include "nvvk/commands_vk.hpp"
#include "nvvk/buffers_vk.hpp"
#include "nvvk/images_vk.hpp"
#include "stb_image.h"
#include "nvh/fileoperations.hpp"
#include "host_device.h"
#include "obj_loader.h"

#include <filesystem>

void ps::pg::ObjLibrary::init(nvvk::ResourceAllocatorDma* ap_alloc, VkDevice a_device, 
                uint32_t a_graphicsQueueIndex, nvvk::DebugUtil& ar_debug){

        mp_alloc = ap_alloc;
        m_device = a_device;
        m_graphicsQueueIndex = a_graphicsQueueIndex;
        mp_debug = &ar_debug;
        m_cmdPool.init(a_device, a_graphicsQueueIndex);
    }

void ps::pg::ObjLibrary::AddMesh(ps::pg::ObjMesh& ar_objMesh, const std::string a_objName){
    ar_objMesh.objIndex = static_cast<uint32_t>(m_meshContainer.size());
    this->m_meshContainer.push_back(ar_objMesh);
    this->m_objectNames.push_back(a_objName);

    return;
}

ps::pg::ObjMesh* ps::pg::ObjLibrary::GetMesh(uint32_t index){
    return (m_meshContainer.size() > index ? &m_meshContainer[index] : nullptr);
}

ps::pg::ObjMesh* ps::pg::ObjLibrary::GetMesh(const std::string& ar_objName){
    uint32_t index = 0;
    for(auto name: this->m_objectNames){
        if(ar_objName == name){
            return &m_meshContainer[index];
        }
        ++index;
    }
    return nullptr;
}

//--------------------------------------------------------------------------------------------------
// Creating all textures and samplers
//
void ps::pg::ObjLibrary::createTextureImages(const VkCommandBuffer& cmdBuf, const std::vector<std::string>& textures)
{
    VkSamplerCreateInfo samplerCreateInfo{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    samplerCreateInfo.minFilter = VK_FILTER_LINEAR;
    samplerCreateInfo.magFilter = VK_FILTER_LINEAR;
    samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerCreateInfo.maxLod = FLT_MAX;

    VkFormat format = VK_FORMAT_R8G8B8A8_SRGB;

    // If no textures are present, create a dummy one to accommodate the pipeline layout
    if (textures.empty() && m_textures.empty())
    {
        nvvk::Texture texture;

        std::array<uint8_t, 4> color{ 255u, 255u, 255u, 255u };
        VkDeviceSize           bufferSize = sizeof(color);
        auto                   imgSize = VkExtent2D{ 1, 1 };
        auto                   imageCreateInfo = nvvk::makeImage2DCreateInfo(imgSize, format);

        // Creating the dummy texture
        nvvk::Image           image = mp_alloc->createImage(cmdBuf, bufferSize, color.data(), imageCreateInfo);
        VkImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, imageCreateInfo);
        texture = mp_alloc->createTexture(image, ivInfo, samplerCreateInfo);

        // The image format must be in VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        nvvk::cmdBarrierImageLayout(cmdBuf, texture.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        m_textures.push_back(texture);
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
                nvvk::Image image = mp_alloc->createImage(cmdBuf, bufferSize, pixels, imageCreateInfo);
                nvvk::cmdGenerateMipmaps(cmdBuf, image.image, format, imgSize, imageCreateInfo.mipLevels);
                VkImageViewCreateInfo ivInfo = nvvk::makeImageViewCreateInfo(image.image, imageCreateInfo);
                nvvk::Texture         texture = mp_alloc->createTexture(image, ivInfo, samplerCreateInfo);

                ps::pg::ObjLibrary::getObjLibrary().m_textures.push_back(texture);
            }

            stbi_image_free(stbi_pixels);
        }
    }
}

void ps::pg::ObjLibrary::LoadMesh(const std::string& ar_file_path, const std::string& ar_name){
    
    const std::string objName = ar_name.empty()? nvh::getFileName(ar_file_path) : ar_name;

    ObjLoader loader;
    loader.loadModel(ar_file_path);

    // Converting from Srgb to linear
    for (auto& m : loader.m_materials)
    {
        m.ambient = nvmath::pow(m.ambient, 2.2f);
        m.diffuse = nvmath::pow(m.diffuse, 2.2f);
        m.specular = nvmath::pow(m.specular, 2.2f);
    }

    ps::pg::ObjMesh model;
    model.nbIndices = static_cast<uint32_t>(loader.m_indices.size());
    model.nbVertices = static_cast<uint32_t>(loader.m_vertices.size());

    // Create the buffers on Device and copy vertices, indices and materials
    VkCommandBuffer    cmdBuf = m_cmdPool.createCommandBuffer();
    VkBufferUsageFlags flag = VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    model.vertexBuffer = mp_alloc->createBuffer(cmdBuf, loader.m_vertices, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | flag);
    model.indexBuffer = mp_alloc->createBuffer(cmdBuf, loader.m_indices, VK_BUFFER_USAGE_INDEX_BUFFER_BIT | flag);
    model.matColorBuffer = mp_alloc->createBuffer(cmdBuf, loader.m_materials, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | flag);
    model.matIndexBuffer = mp_alloc->createBuffer(cmdBuf, loader.m_matIndx, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | flag);

    // Creates all textures found and find the offset for this model
    auto txtOffset = static_cast<uint32_t>(m_textures.size());
    createTextureImages(cmdBuf, loader.m_textures);
    m_cmdPool.submitAndWait(cmdBuf);
    mp_alloc->finalizeAndReleaseStaging();

    std::string objNb = std::to_string(m_meshContainer.size());
    mp_debug->setObjectName(model.vertexBuffer.buffer, (std::string("vertex_" + objNb)));
    mp_debug->setObjectName(model.indexBuffer.buffer, (std::string("index_" + objNb)));
    mp_debug->setObjectName(model.matColorBuffer.buffer, (std::string("mat_" + objNb)));
    mp_debug->setObjectName(model.matIndexBuffer.buffer, (std::string("matIdx_" + objNb)));

    // Creating information for device access
    ObjDesc desc;
    desc.txtOffset = 0;
    desc.txtOffset = txtOffset;
    desc.vertexAddress = nvvk::getBufferDeviceAddress(m_device, model.vertexBuffer.buffer);
    desc.indexAddress = nvvk::getBufferDeviceAddress(m_device, model.indexBuffer.buffer);
    desc.materialAddress = nvvk::getBufferDeviceAddress(m_device, model.matColorBuffer.buffer);
    desc.materialIndexAddress = nvvk::getBufferDeviceAddress(m_device, model.matIndexBuffer.buffer);

    // Keeping the obj host model and device description
    this->AddMesh(model, objName);
    m_descContainer.emplace_back(desc);
    return;
}

void ps::pg::ObjLibrary::LoadDirectory(const std::string& ar_directory_path, std::vector<std::string>& ar_names){

    uint32_t index = 0;
    std::string objName = "";

    std::filesystem::path input_directory(ar_directory_path);
    for (const auto& dir_entry : std::filesystem::directory_iterator{input_directory}) 
    {
        auto test = dir_entry.path().extension().string();
        if(dir_entry.path().extension().string() != ".obj")
            continue;

        if(ar_names.empty() || ar_names.size() <= index || ar_names[index].empty())
            objName = dir_entry.path().stem().string(); // this almost looks like Java code. Yep, that's why c++ > 11 is a joke.
        else
            objName = ar_names[index];
        
        LoadMesh(dir_entry.path().string(), objName);
    }

    return;
}

void ps::pg::ObjLibrary::CreateObjDescriptionBuffer(){
    nvvk::CommandPool cmdGen(m_device, m_graphicsQueueIndex);

    auto cmdBuf = cmdGen.createCommandBuffer();
    m_bObjDesc = mp_alloc->createBuffer(cmdBuf, m_descContainer, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    cmdGen.submitAndWait(cmdBuf);
    mp_alloc->finalizeAndReleaseStaging();
    mp_debug->setObjectName(m_bObjDesc.buffer, "ObjDescs");
}