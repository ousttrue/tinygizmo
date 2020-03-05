set(TARGET_NAME tinygizmo)
add_library(${TARGET_NAME}
    tinygizmo.cpp
    utilmath.cpp
    geometry_mesh.cpp
    gizmo_position.cpp
    gizmo_orientation.cpp
    gizmo_scale.cpp
    )
set_property(TARGET ${TARGET_NAME}
    PROPERTY CXX_STANDARD 20
    )
target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../castalg/include
    )
target_link_libraries(${TARGET_NAME} PUBLIC
    screenstate
    )
