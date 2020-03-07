set(TARGET_NAME tinygizmo_example_d3d11)
add_executable(${TARGET_NAME}
    main.cpp
    renderer_impl_d3d11.cpp
    dx11_context.cpp
    )
set_property(TARGET ${TARGET_NAME}
    PROPERTY CXX_STANDARD 20
    )
target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../tinygizmo_example
    )
target_compile_definitions(${TARGET_NAME} PRIVATE
    )
target_link_libraries(${TARGET_NAME} PRIVATE
    tinygizmo
    d3d11
    d3dcompiler
    )
