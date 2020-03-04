#pragma once
#include <array>

struct CameraView
{
    std::array<float, 3> position;

    float pitch = 0;
    float yaw = 0;
    std::array<float, 3> shift{0, -1.5f, -4};

    std::array<float, 16> matrix{};
    std::array<float, 4> orientation{};

    int m_lastMouseX = -1;
    int m_lastMouseY = -1;
    void update(float deltaTime, int mouseX, int mouseY,
                bool mouseRightDown, bool mouseMiddleDown, int mouseWheel);
};

struct CameraProjection
{
    float yfov = 1.0f; // 0.3 PI ?
    float near_clip = 0.01f;
    float far_clip = 32.0f;

    std::array<float, 16> matrix{};
    void update(float aspectRatio);
};
