project(pga)

message(STATUS "++++++++++++++++++++++++++++")
message(STATUS "[PINK] creating ${PROJECT_NAME} target")
message(STATUS "++++++++++++++++++++++++++++")

file(GLOB SOURCE_FILES src/*.cpp)

add_library(${PROJECT_NAME} SHARED)   

target_sources(${PROJECT_NAME}
    PUBLIC ${SOURCE_FILES}
    PUBLIC ${COMMON_SOURCE_FILES}
)

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    INTERFACE nvpro_core
)