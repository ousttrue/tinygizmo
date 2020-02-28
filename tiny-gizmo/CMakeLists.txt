set(TARGET_NAME tiny-gizmo)
add_library(${TARGET_NAME}
    tiny-gizmo.cpp
    )
target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../castalg/include
    )
