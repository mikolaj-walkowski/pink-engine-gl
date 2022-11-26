project(pink_graphics)

add_library(${PROJECT_NAME} src/pink_graphics.cpp)
add_library(pink::graphics ALIAS ${PROJECT_NAME})
_add_project_definitions(${PROJECT_NAME})


target_link_libraries(${PROJECT_NAME}
    pink::structs
)

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    ${glm_SOURCE_DIRS}
)