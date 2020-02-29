set(TARGET_NAME tiny-gizmo)
add_library(${TARGET_NAME}
    tiny-gizmo.cpp
    tiny-gizmo_interaction_state.cpp
    utilmath.cpp
    geometry_mesh.cpp
    position_gizmo.cpp
    )
set_property(TARGET ${TARGET_NAME}
    PROPERTY CXX_STANDARD 20
    )
target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../castalg/include
    )
