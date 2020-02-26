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
        if (key < 128)
            w->m_state.keycode[key] = (action != GLFW_RELEASE);
        else if (key == GLFW_KEY_ESCAPE)
            w->close();
        else if (key == GLFW_KEY_LEFT_CONTROL)
            w->m_state.key_left_control = (action != GLFW_RELEASE);
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

    glfwSetWindowSizeCallback(window, [](GLFWwindow *window, int width, int height) {
        auto w = (Window *)glfwGetWindowUserPointer(window);
        w->m_state.windowWidth = width;
        w->m_state.windowHeight = height;
    });
    glfwGetWindowSize(window, &m_state.windowWidth, &m_state.windowHeight);

    glfwSetWindowUserPointer(window, this);

    glfwSwapInterval(1);

    return true;
}

void Window::swap_buffers() { glfwSwapBuffers(window); }

void Window::close() { glfwSetWindowShouldClose(window, 1); }

bool Window::loop(WindowState *pState)
{
    if (glfwWindowShouldClose(window))
    {
        return false;
    }

    glfwPollEvents();
    *pState = m_state;

    return true;
}
