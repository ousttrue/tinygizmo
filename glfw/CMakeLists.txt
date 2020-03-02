set(TARGET_NAME glfw)
set(GLFW_DIR ${CMAKE_CURRENT_LIST_DIR}/../tinygizmo_example/third-party/glfw-3.1.2)
add_library(${TARGET_NAME}
    ${GLFW_DIR}/src/context.c
    ${GLFW_DIR}/src/init.c
    ${GLFW_DIR}/src/input.c
    ${GLFW_DIR}/src/monitor.c
    ${GLFW_DIR}/src/wgl_context.c
    ${GLFW_DIR}/src/win32_init.c
    ${GLFW_DIR}/src/win32_monitor.c
    ${GLFW_DIR}/src/win32_time.c
    ${GLFW_DIR}/src/win32_tls.c
    ${GLFW_DIR}/src/win32_window.c
    ${GLFW_DIR}/src/window.c
    ${GLFW_DIR}/src/winmm_joystick.c
    )
target_include_directories(${TARGET_NAME} PUBLIC
    ${GLFW_DIR}/include
    )
target_compile_definitions(${TARGET_NAME} PUBLIC
    _GLFW_USE_CONFIG_H
    # GLFW_INCLUDE_GLCOREARB
    )
