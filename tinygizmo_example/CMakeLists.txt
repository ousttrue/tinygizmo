set(TARGET_NAME tinygizmo_example)
add_executable(${TARGET_NAME}
    main.cpp
    gl-api.cpp
    renderer_impl_gl3.cpp
    wgl_context.cpp
    )
set_property(TARGET ${TARGET_NAME}
    PROPERTY CXX_STANDARD 20
    )
target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/third-party
    ${CMAKE_CURRENT_LIST_DIR}/third-party/glew/include
    ${CMAKE_CURRENT_LIST_DIR}/third-party/glfw-3.1.2/include
    ${CMAKE_CURRENT_LIST_DIR}/../castalg/include
    )
target_compile_definitions(${TARGET_NAME} PRIVATE
    )
target_link_libraries(${TARGET_NAME} PRIVATE
    tinygizmo
    ${CMAKE_CURRENT_LIST_DIR}/third-party/glew/lib/x64/glew32s.lib
    OPENGL32
    )
