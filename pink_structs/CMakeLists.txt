project(pink_structs)

message(STATUS "++++++++++++++++++++++++++++")
message(STATUS "[PINK] creating ${PROJECT_NAME} target")
message(STATUS "++++++++++++++++++++++++++++")

add_library(${PROJECT_NAME} INTERFACE)
add_library(pink::structs ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
    # INTERFACE klein::klein
    INTERFACE pga
    INTERFACE nvpro_core
    # INTERFACE assimp
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${PROJECT_SOURCE_DIR}/include
)
