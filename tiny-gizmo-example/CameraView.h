#pragma once
#include "window.h"
#include <array>


struct CameraView
{
    std::array<float, 3> position;

    float pitch = 0;
    float yaw = 0;
    std::array<float, 3> shift;

    std::array<float, 16> matrix{};
    std::array<float, 4> orientation{};

    WindowState lastState{};
    void update(WindowState &state);
};

struct CameraProjection
{
    std::array<float, 16> matrix{};
    void update(float yfov, float aspectRatio, float near_clip, float far_clip);
};
