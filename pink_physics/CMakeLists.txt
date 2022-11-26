project(pink_physics)

message(STATUS "Adding ${PROJECT_NAME} ")

file(GLOB SOURCE_FILES src/*.cpp)

add_library(${PROJECT_NAME} SHARED)   
add_library(pink::physics ALIAS ${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PUBLIC ${COMMON_SOURCE_FILES}
    PUBLIC ${SOURCE_FILES}
)


target_link_libraries(${PROJECT_NAME}
    pink::structs
    klein::klein
    nvpro_core
)

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)
