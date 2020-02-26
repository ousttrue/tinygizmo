#pragma once
#include <array>
#include <chrono>

struct WindowState
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
    // time state
    std::chrono::steady_clock::time_point time{};
};

class Window
{
    struct GLFWwindow *window = nullptr;

public:
    WindowState m_state;

    Window();
    ~Window();

    Window(const Window &) = delete;
    Window(Window &&) = delete;
    Window &operator=(const Window &) = delete;
    Window &operator=(Window &&) = delete;

    bool initialize(int width, int height, const char *title);
    bool loop(WindowState *pState);
    void swap_buffers();
    void close();
};
