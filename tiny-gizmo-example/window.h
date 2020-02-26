#pragma once
#include <array>
#include <linalg.h>

class Window
{
    struct GLFWwindow *window = nullptr;

public:
    struct State
    {
        // window state
        int windowWidth = 0;
        int windowHeight = 0;
        // mouse state
        int mouseX = 0;
        int mouseY = 0;
        bool mouseLeftDown = 0;
        bool mouseRightDown = 0;
        // key state
        std::array<bool, 127> keycode{};
        bool key_left_control = false;
    };
    State m_state;

    Window();
    ~Window();

    Window(const Window &) = delete;
    Window(Window &&) = delete;
    Window &operator=(const Window &) = delete;
    Window &operator=(Window &&) = delete;

    bool initialize(int width, int height, const char *title);
    bool loop(State *pState);
    void swap_buffers();

private:
    struct GLFWwindow *get_glfw_window_handle();
    bool should_close() const;
    int get_window_attrib(int attrib) const;
    linalg::aliases::int2 get_window_size() const;
    void set_window_size(linalg::aliases::int2 newSize);
    linalg::aliases::int2 get_framebuffer_size() const;
    linalg::aliases::float2 get_cursor_pos() const;

    void close();
};
