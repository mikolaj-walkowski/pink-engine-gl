project(pink_graphics)

file(GLOB SOURCE_FILES src/*.cpp)

add_library(${PROJECT_NAME} SHARED)
message(STATUS "++++++++++++++++++++++++++++")
message(STATUS "[PINK] Creating compile shaders target")
message(STATUS "++++++++++++++++++++++++++++")

set(SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/res)
set(SHADER_DST_DIR ${OUTPUT_PATH}/spv )

file(MAKE_DIRECTORY "${SHADER_DST_DIR}")

message(STATUS "    Shader source dir: ${SHADER_DIR}")

message(STATUS "    Shader build dir: ${SHADER_DST_DIR}")

compile_glsl_directory(
    SRC "${SHADER_DIR}"
	DST "${SHADER_DST_DIR}"
	VULKAN_TARGET "vulkan1.2"
	DEPENDENCY ${VULKAN_BUILD_DEPENDENCIES}
	)
    
add_custom_target(compiled_shaders DEPENDS ${SPV_OUTPUT})
    
add_dependencies(${PROJECT_NAME} compiled_shaders)

message(STATUS "++++++++++++++++++++++++++++")
message(STATUS "[PINK] Adding ${PROJECT_NAME} target")
message(STATUS "++++++++++++++++++++++++++++")

add_library(pink::graphics ALIAS ${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PUBLIC ${COMMON_SOURCE_FILES}
    PUBLIC ${SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME}
    pink::structs
    nvpro_core
)

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    PUBLIC ${PROJECT_SOURCE_DIR}/res
)