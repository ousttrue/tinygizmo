#include "window.h"
#define GLEW_STATIC
#define GL_GLEXT_PROTOTYPES
#include <glew.h>

#define GLFW_INCLUDE_GLU
#include <GLFW\glfw3.h>

Window::Window()
{
}

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
    });

    glfwSetKeyCallback(window, [](GLFWwindow *window, int key, int, int action, int mods) {
        auto w = (Window *)glfwGetWindowUserPointer(window);
        if (w->on_key)
            w->on_key(key, action, mods);
    });

    glfwSetMouseButtonCallback(window, [](GLFWwindow *window, int button, int action, int mods) {
        auto w = (Window *)glfwGetWindowUserPointer(window);
        if (button == GLFW_MOUSE_BUTTON_LEFT)
            w->m_state.mouseLeftDown = (action != GLFW_RELEASE);
        if (button == GLFW_MOUSE_BUTTON_RIGHT)
            w->m_state.mouseRightDown = (action != GLFW_RELEASE);
    });

    glfwSetCursorPosCallback(window, [](GLFWwindow *window, double x, double y) {
        auto w = (Window *)glfwGetWindowUserPointer(window);
        w->m_state.mouseX = (int)x;
        w->m_state.mouseY = (int)y;
    });

    glfwSetDropCallback(window, [](GLFWwindow *window, int numFiles, const char **paths) {
        auto w = (Window *)glfwGetWindowUserPointer(window);
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
