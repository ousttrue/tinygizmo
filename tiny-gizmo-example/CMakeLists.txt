set(TARGET_NAME tiny-gizmo-example)
add_executable(${TARGET_NAME}
    example-main.cpp
    gl-api.cpp
    )
set_property(TARGET ${TARGET_NAME}
    PROPERTY CXX_STANDARD 20
    )
target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/third-party
    ${CMAKE_CURRENT_LIST_DIR}/third-party/glew
    ${CMAKE_CURRENT_LIST_DIR}/third-party/glfw-3.1.2/include
    )
target_compile_definitions(${TARGET_NAME} PRIVATE
    # GLFW_INCLUDE_GLCOREARB
    )
target_link_libraries(${TARGET_NAME} PRIVATE
    glfw
    tiny-gizmo
    ${CMAKE_CURRENT_LIST_DIR}/third-party/glew/lib/x64/glew32s.lib
    # ${CMAKE_CURRENT_LIST_DIR}/third-party/glew/lib/x64/glew32.lib
    OPENGL32
    )
