cmake_minimum_required(VERSION 3.4.0)
project(pink_engine VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(pink_engine main.cpp)

find_package(OpenGL REQUIRED)


add_subdirectory(glm EXCLUDE_FROM_ALL)
add_subdirectory(glfw EXCLUDE_FROM_ALL)

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
                    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
                    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
                )

target_include_directories( IMGUI
                            PUBLIC ${IMGUI_DIR}
                            PUBLIC ${IMGUI_DIR}/backends
                            PUBLIC ${GLFW_SOURCE_DIR}/include
                            )

target_link_libraries(IMGUI PUBLIC glfw ${CMAKE_DL_LIBS} ${OPENGL_LIBRARIES})

add_subdirectory(pink_structs)
add_subdirectory(pink_physics)
add_subdirectory(pink_graphics)


include_directories(
        ${OPENGL_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/assimp/include
        ${PROJECT_SOURCE_DIR}
        )


target_link_libraries(pink_engine
        ${OPENGL_LIBRARIES}
        glm
        glfw
        libglew_static 
        IMGUI
        assimp # ${CMAKE_CURRENT_BINARY_DIR}/assimp/lib/Debug/assimp-vc143-mtd.lib
        ${CMAKE_DL_LIBS}
        pink::structs
        pink::physics
        pink::graphics
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
