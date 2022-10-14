cmake_minimum_required(VERSION 3.4.0)
project(pink_engine VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(pink_engine main.cpp)

set (VULKAN_SDK_PATH "C:\\VulkanSDK\\1.3.216.0")

message(STATUS "VULKAN $ENV{VK_SDK_PATH}")
if (DEFINED ENV{VK_SDK_PATH})
  set(Vulkan_INCLUDE_DIRS "$ENV{VK_SDK_PATH}/Include") # 1.1 Make sure this include path is correct
  set(Vulkan_LIBRARIES "$ENV{VK_SDK_PATH}/Lib") # 1.2 Make sure lib path is correct
  set(Vulkan_FOUND "True")
else()
  find_package(Vulkan) # throws error if could not find Vulkan
  message(STATUS "Found Vulkan: $ENV{VULKAN_SDK}")
endif()
if (NOT Vulkan_FOUND)
    message(FATAL_ERROR "Could not find Vulkan library!")
else()
    message(STATUS "Using vulkan lib at: ${Vulkan_LIBRARIES}")
endif()

add_subdirectory(glm EXCLUDE_FROM_ALL)
add_subdirectory(glfw EXCLUDE_FROM_ALL)
add_subdirectory(vk-bootstrap EXCLUDE_FROM_ALL)
#add_subdirectory(VulkanMemoryAllocator EXCLUDE_FROM_ALL)

set(ASSIMP_BUILD_STATIC_LIB ON)
set(ASSIMP_INSTALL OFF)
set(ASSIMP_TOOLS ON)
set(ENABLE_BOOST_WORKAROUND ON)
add_subdirectory(assimp EXCLUDE_FROM_ALL)

add_subdirectory(glew-cmake EXCLUDE_FROM_ALL)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
add_library(IMGUI STATIC)

target_sources( IMGUI
                PRIVATE
                    ${IMGUI_DIR}/imgui_demo.cpp
                    ${IMGUI_DIR}/imgui_draw.cpp
                    ${IMGUI_DIR}/imgui_tables.cpp
                    ${IMGUI_DIR}/imgui_widgets.cpp
                    ${IMGUI_DIR}/imgui.cpp

                PRIVATE
                    ${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp
                    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
                )

target_include_directories( IMGUI
                            PUBLIC ${IMGUI_DIR}
                            PUBLIC ${IMGUI_DIR}/backends
                            PUBLIC ${GLFW_SOURCE_DIR}/include
                            PUBLIC ${Vulkan_INCLUDE_DIRS}
                            )

target_link_directories( IMGUI PUBLIC
                        ${Vulkan_LIBRARIES} 
)

target_link_libraries(IMGUI PUBLIC glfw ${CMAKE_DL_LIBS})

add_subdirectory(pink_structs)
add_subdirectory(pink_physics)
add_subdirectory(pink_graphics)


target_include_directories(pink_engine PUBLIC
        ${Vulkan_INCLUDE_DIRS}
        )

target_link_directories( pink_engine PUBLIC
        ${Vulkan_LIBRARIES} 
)

target_link_libraries(pink_engine
        glm
        glfw
        libglew_static 
        IMGUI
        assimp
        ${CMAKE_DL_LIBS}
        pink::structs
        pink::physics
        pink::graphics
        vk-bootstrap::vk-bootstrap
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
