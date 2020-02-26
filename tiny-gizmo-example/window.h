#pragma once
#include <functional>
#include <linalg.h>

class Window
{
    struct GLFWwindow *window = nullptr;

public:
    std::function<void(unsigned int codepoint)> on_char;
    std::function<void(int key, int action, int mods)> on_key;
    std::function<void(int button, int action, int mods)> on_mouse_button;
    std::function<void(linalg::aliases::float2 pos)> on_cursor_pos;
    std::function<void(int numFiles, const char **paths)> on_drop;

    struct State
    {
        // mouse state
        bool ml = 0, mr = 0;
        // wasd state
        bool bf = 0, bl = 0, bb = 0, br = 0;
    };
    State m_state;

    Window(int width, int height, const char *title);
    ~Window();

    Window(const Window &) = delete;
    Window(Window &&) = delete;
    Window &operator=(const Window &) = delete;
    Window &operator=(Window &&) = delete;

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
