#pragma once
#include "window.h"
#include <array>

namespace tinygizmo
{
struct camera_parameters;
}

struct GuiCamera
{
    float pitch = 0;
    float yaw = 0;
    std::array<float, 3> ray_dir{};
    std::array<float, 16> view_proj_matrix{};

    WindowState lastState{};
    void update(WindowState &state,
                tinygizmo::camera_parameters &params);
};
