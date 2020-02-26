#include "window.h"
#define GLEW_STATIC
#define GL_GLEXT_PROTOTYPES
#include <glew.h>

#define GLFW_INCLUDE_GLU
#include <GLFW\glfw3.h>

Window::Window()
{}

Window::~Window()
{
    glfwMakeContextCurrent(window);
    glfwDestroyWindow(window);
    glfwTerminate();
}

bool Window::initialize(int width, int height, const char *title)
{
    if (glfwInit() == GL_FALSE)
        throw std::runtime_error("glfwInit() failed");
    window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window)
        throw std::runtime_error("glfwCreateWindow() failed");

    glfwMakeContextCurrent(window);

    if (GLenum err = glewInit())
    {
        throw std::runtime_error(std::string("glewInit() failed - ") + (const char *)glewGetErrorString(err));
    }

    glfwSetCharCallback(window, [](GLFWwindow *window, unsigned int codepoint) {
        auto w = (Window *)glfwGetWindowUserPointer(window);
        if (w->on_char)
            w->on_char(codepoint);
    });

    glfwSetKeyCallback(window, [](GLFWwindow *window, int key, int, int action, int mods) {
        auto w = (Window *)glfwGetWindowUserPointer(window);
        if (w->on_key)
            w->on_key(key, action, mods);
    });

    glfwSetMouseButtonCallback(window, [](GLFWwindow *window, int button, int action, int mods) {
        auto w = (Window *)glfwGetWindowUserPointer(window);
        if (w->on_mouse_button)
            w->on_mouse_button(button, action, mods);
    });

    glfwSetCursorPosCallback(window, [](GLFWwindow *window, double xpos, double ypos) {
        auto w = (Window *)glfwGetWindowUserPointer(window);
        if (w->on_cursor_pos)
            w->on_cursor_pos(linalg::aliases::float2(linalg::aliases::double2(xpos, ypos)));
    });

    glfwSetDropCallback(window, [](GLFWwindow *window, int numFiles, const char **paths) {
        auto w = (Window *)glfwGetWindowUserPointer(window);
        if (w->on_drop)
            w->on_drop(numFiles, paths);
    });

    glfwSetWindowUserPointer(window, this);

    glfwSwapInterval(1);

    return true;
}

GLFWwindow *Window::get_glfw_window_handle() { return window; };
bool Window::should_close() const { return !!glfwWindowShouldClose(window); }
int Window::get_window_attrib(int attrib) const { return glfwGetWindowAttrib(window, attrib); }
linalg::aliases::int2 Window::get_window_size() const
{
    linalg::aliases::int2 size;
    glfwGetWindowSize(window, &size.x, &size.y);
    return size;
}
void Window::set_window_size(linalg::aliases::int2 newSize) { glfwSetWindowSize(window, newSize.x, newSize.y); }
linalg::aliases::int2 Window::get_framebuffer_size() const
{
    linalg::aliases::int2 size;
    glfwGetFramebufferSize(window, &size.x, &size.y);
    return size;
}
linalg::aliases::float2 Window::get_cursor_pos() const
{
    linalg::aliases::double2 pos;
    glfwGetCursorPos(window, &pos.x, &pos.y);
    return linalg::aliases::float2(pos);
}

void Window::swap_buffers() { glfwSwapBuffers(window); }
void Window::close() { glfwSetWindowShouldClose(window, 1); }

bool Window::loop(State *pState)
{
    if (should_close())
    {
        return false;
    }

    glfwPollEvents();
    *pState = m_state;

    return true;
}
