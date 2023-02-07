project(pink_core)

message(STATUS "++++++++++++++++++++++++++++")
message(STATUS "[PINK] creating ${PROJECT_NAME} target")
message(STATUS "++++++++++++++++++++++++++++")

file(GLOB SOURCE_FILES src/*.cpp)

add_library(${PROJECT_NAME} SHARED)   
add_library(pink::core ALIAS ${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PUBLIC ${COMMON_SOURCE_FILES}
    PUBLIC ${SOURCE_FILES}
)


target_link_libraries(${PROJECT_NAME}
    pink::structs
    klein::klein
    nvpro_core
    imgui
)

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)