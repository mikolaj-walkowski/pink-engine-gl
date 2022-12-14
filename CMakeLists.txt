cmake_minimum_required(VERSION 3.4.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

get_filename_component(PROJNAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/out)

project(pink_engine VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()
message(STATUS "++++++++++++++++++++++++++++")
message(STATUS "[PINK] Adding klein")
message(STATUS "++++++++++++++++++++++++++++")
add_subdirectory(klein EXCLUDE_FROM_ALL)


if(MSVC)
set_target_properties(klein PROPERTIES COMPILE_FLAGS "/w")
else()
set_target_properties(klein PROPERTIES COMPILE_FLAGS "-w")
endif()

# set(ASSIMP_BUILD_STATIC_LIB ON)
# set(ASSIMP_INSTALL OFF)
# set(ASSIMP_TOOLS ON)
# set(ENABLE_BOOST_WORKAROUND ON)
# set(ASSIMP_BUILD_TESTS OFF)
# set(ASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT OFF)
# set(ASSIMP_NO_EXPORT OFF)

# message(STATUS "++++++++++++++++++++++++++++")
# message(STATUS "[PINK] Adding asimp")
# message(STATUS "++++++++++++++++++++++++++++")

# add_subdirectory(assimp EXCLUDE_FROM_ALL)

message(STATUS "++++++++++++++++++++++++++++")
message(STATUS "[PINK] Adding nnvpro_core")
message(STATUS "++++++++++++++++++++++++++++")

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

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/media DESTINATION ${OUTPUT_PATH})


message(STATUS "++++++++++++++++++++++++++++")
message(STATUS "[PINK] Creating ${PROJECT_NAME} target")
message(STATUS "++++++++++++++++++++++++++++")

add_executable(pink_engine)

_add_project_definitions(${PROJECT_NAME})

add_subdirectory(pga EXCLUDE_FROM_ALL)
add_subdirectory(pink_structs)
add_subdirectory(pink_graphics)
add_subdirectory(pink_physics)

  file(GLOB SOURCES  ${PROJECT_SOURCE_DIR}/src/*.cpp)

  target_sources(${PROJECT_NAME} 
        PUBLIC ${SOURCES}
        PUBLIC ${COMMON_SOURCE_FILES}
        PUBLIC ${GLSL_HEADERS} ${GLSL_SOURCES}
  )
  
  target_include_directories(${PROJECT_NAME} 
        PUBLIC  ${PROJECT_SOURCE_DIR}/include
  )


target_link_libraries(${PROJECT_NAME}
        #assimp
      #   klein::klein
        pga
        nvpro_core
        pink::structs
        pink::graphics
        pink::physics
)

_finalize_target(pink_engine)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
