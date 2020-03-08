#pragma once
#include "ScreenState.h"
#include "CameraState.h"
#include <array>

enum class PerspectiveTypes
{
    OpenGL,
    D3D,
};

struct OrbitCamera
{
    camera::CameraState state;

    PerspectiveTypes perspectiveType;

    float zNear = 0.1f;
    float zFar = 100.0f;
    float aspectRatio = 1.0f;

    int prevMouseX = -1;
    int prevMouseY = -1;

    std::array<float, 3> shift{0, 0.2f, -4};
    float yawRadians = 0;
    float pitchRadians = 0;

    OrbitCamera(PerspectiveTypes type = PerspectiveTypes::OpenGL)
        : perspectiveType(type)
    {
    }
    void CalcView(int w, int h, int x, int y);
    void CalcPerspective();
    void SetViewport(int x, int y, int w, int h);
    void WindowInput(const screenstate::ScreenState &window);
};
