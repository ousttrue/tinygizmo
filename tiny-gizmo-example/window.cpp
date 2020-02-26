#include "window.h"



bool Window::loop(State *pState)
{
    if(should_close())
    {
        return false;
    }

    glfwPollEvents();
    *pState = m_state;

    return true;
}
