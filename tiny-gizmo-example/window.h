#pragma once
#include <functional>
#include <linalg.h>

class Window
{
    struct GLFWwindow *window = nullptr;

public:
    std::function<void(int key, int action, int mods)> on_key;

    struct State
    {
        // mouse state
        int mouseX=0;
        int mouseY=0;
        bool mouseLeftDown = 0;
        bool mouseRightDown = 0;
        // wasd state
        bool bf = 0;
        bool bl = 0;
        bool bb = 0;
        bool br = 0;
    };
    State m_state;

    Window();
    ~Window();

    Window(const Window &) = delete;
    Window(Window &&) = delete;
    Window &operator=(const Window &) = delete;
    Window &operator=(Window &&) = delete;

    bool initialize(int width, int height, const char *title);
    struct GLFWwindow *get_glfw_window_handle();
    bool should_close() const;
    int get_window_attrib(int attrib) const;
    linalg::aliases::int2 get_window_size() const;
    void set_window_size(linalg::aliases::int2 newSize);
    linalg::aliases::int2 get_framebuffer_size() const;
    linalg::aliases::float2 get_cursor_pos() const;

    void swap_buffers();
    void close();
    bool loop(State *pState);
};
