cmake_minimum_required(VERSION 3.4.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # works

get_filename_component(PROJNAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(PROJNAME vk_${PROJNAME}_KHR)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/out)

project(pink_engine VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

message(STATUS "Adding klein")

add_subdirectory(klein EXCLUDE_FROM_ALL)

set(ASSIMP_BUILD_STATIC_LIB ON)
set(ASSIMP_INSTALL OFF)
set(ASSIMP_TOOLS ON)
set(ENABLE_BOOST_WORKAROUND ON)
set(ASSIMP_BUILD_TESTS OFF)
set(ASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT OFF)
set(ASSIMP_NO_EXPORT OFF)

message(STATUS "Adding asimp")

add_subdirectory(assimp EXCLUDE_FROM_ALL)

message(STATUS "Adding nnvpro_core")
set(BASE_DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR})

if(MSVC)
add_definitions(/wd26812)  # 'enum class' over 'enum'
add_definitions(/wd26451)  # Arithmetic overflow, casting 4 byte value to 8 byte value
endif()

include(${BASE_DIRECTORY}/nvpro_core/cmake/setup.cmake)
include(${BASE_DIRECTORY}/nvpro_core/cmake/utilities.cmake)

_add_package_VulkanSDK()
_add_package_IMGUI()
_add_package_ShaderC()

_add_nvpro_core_lib()

message(${OUTPUT_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/media DESTINATION ${OUTPUT_PATH})

compile_glsl_directory(
  SRC "${CMAKE_CURRENT_SOURCE_DIR}/shaders" 
	DST "${CMAKE_CURRENT_SOURCE_DIR}/spv" 
	VULKAN_TARGET "vulkan1.2"
	DEPENDENCY ${VULKAN_BUILD_DEPENDENCIES}
	)
  
  add_executable(pink_engine)
  
  _add_project_definitions(${PROJECT_NAME})

  file(GLOB SOURCES  ${PROJECT_SOURCE_DIR}/src/*.cpp)

  target_sources(${PROJECT_NAME} 
        PUBLIC ${SOURCES}
        PUBLIC ${COMMON_SOURCE_FILES}
  )
  
  target_include_directories(${PROJECT_NAME} 
        PUBLIC ${PROJECT_SOURCE_DIR}/shaders
        PUBLIC  ${PROJECT_SOURCE_DIR}/include
  )

add_subdirectory(pink_structs)
add_subdirectory(pink_graphics)
add_subdirectory(pink_physics)

target_link_libraries(${PROJECT_NAME}
        assimp
        klein::klein
        nvpro_core
        pink::structs
        pink::graphics
        pink::physics
)

_finalize_target(pink_engine)

install(FILES ${SPV_OUTPUT} CONFIGURATIONS Release DESTINATION "bin_${ARCH}/${PROJNAME}/spv")
install(FILES ${SPV_OUTPUT} CONFIGURATIONS Debug DESTINATION "bin_${ARCH}_debug/${PROJNAME}/spv")


install(DIRECTORY "media" 
        CONFIGURATIONS Release 
        DESTINATION "bin_${ARCH}")
install(DIRECTORY "media" 
        CONFIGURATIONS Debug 
        DESTINATION "bin_${ARCH}_debug")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
