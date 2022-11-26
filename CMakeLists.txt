cmake_minimum_required(VERSION 3.4.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # works

project(pink_engine VERSION 0.1.0)

message(${CMAKE_SIZEOF_VOID_P})
include(CTest)
enable_testing()



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
# add_subdirectory(glfw EXCLUDE_FROM_ALL)
# add_subdirectory(vk-bootstrap EXCLUDE_FROM_ALL)
add_subdirectory(klein EXCLUDE_FROM_ALL)

set(ASSIMP_BUILD_STATIC_LIB ON)
set(ASSIMP_INSTALL OFF)
set(ASSIMP_TOOLS ON)
set(ENABLE_BOOST_WORKAROUND ON)
set(ASSIMP_BUILD_TESTS OFF)
set(ASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT OFF)
set(ASSIMP_NO_EXPORT OFF)

add_subdirectory(assimp EXCLUDE_FROM_ALL)

set(USING_OPENGL OFF)
set(USING_VULKANSDK  ON)
set(USING_IMGUI ON)
set(USING_DIRECTX12 OFF)
set(USING_OPTIX ON)
set(BASE_DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR})

include(${BASE_DIRECTORY}/nvpro_core/cmake/setup.cmake)
include(${BASE_DIRECTORY}/nvpro_core/cmake/utilities.cmake)

if(MSVC)
add_definitions(/wd26812)  # 'enum class' over 'enum'
add_definitions(/wd26451)  # Arithmetic overflow, casting 4 byte value to 8 byte value
endif()

set(TUTO_KHR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

_add_nvpro_core_lib()


set(ASSIMP_BUILD_STATIC_LIB ON)
set(ASSIMP_INSTALL OFF)
set(ASSIMP_TOOLS ON)
set(ENABLE_BOOST_WORKAROUND ON)
set(ASSIMP_BUILD_TESTS OFF)
set(ASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT OFF)
set(ASSIMP_NO_EXPORT OFF)

add_subdirectory(glew-cmake EXCLUDE_FROM_ALL)

# set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
# add_library(imgui STATIC)

# target_sources( imgui
#                 PRIVATE
#                     ${IMGUI_DIR}/imgui_demo.cpp
#                     ${IMGUI_DIR}/imgui_draw.cpp
#                     ${IMGUI_DIR}/imgui_tables.cpp
#                     ${IMGUI_DIR}/imgui_widgets.cpp
#                     ${IMGUI_DIR}/imgui.cpp

#                 PRIVATE
#                     ${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp
#                     ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
#                 )

# target_include_directories( imgui
#                             PUBLIC ${IMGUI_DIR}
#                             PUBLIC ${IMGUI_DIR}/backends
#                             PUBLIC ${GLFW_SOURCE_DIR}/include
#                             PUBLIC ${Vulkan_INCLUDE_DIRS}
#                             )

# target_link_directories( imgui PUBLIC
#                         ${Vulkan_LIBRARIES} 
# )

# target_link_libraries(imgui PUBLIC glfw ${CMAKE_DL_LIBS})

add_executable(pink_engine ${PROJECT_SOURCE_DIR}/main.cpp)
add_subdirectory(pink_structs)
add_subdirectory(pink_graphics)
add_subdirectory(pink_physics)


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
        imgui
        assimp
        pink::structs
        pink::physics
        pink::graphics
        klein::klein
        # vk-bootstrap::vk-bootstrap
        nvpro_core
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
