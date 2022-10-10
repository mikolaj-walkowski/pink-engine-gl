project(pink_physics)

add_library(${PROJECT_NAME} src/pink_physics.cpp)
add_library(pink::physics ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
    pink::structs
)

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)