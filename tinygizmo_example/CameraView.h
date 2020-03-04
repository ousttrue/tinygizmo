#pragma once
#include "window.h"
#include <array>

struct CameraView
{
    std::array<float, 3> position;

    float pitch = 0;
    float yaw = 0;
    std::array<float, 3> shift{0, -1.5f, -4};

    std::array<float, 16> matrix{};
    std::array<float, 4> orientation{};

    WindowState lastState{};
    void update(WindowState &state);
};

struct CameraProjection
{
    float yfov = 1.0f; // 0.3 PI ?
    float near_clip = 0.01f;
    float far_clip = 32.0f;

    std::array<float, 16> matrix{};
    void update(float aspectRatio);
};
