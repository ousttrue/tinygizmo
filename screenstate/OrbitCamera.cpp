#include "OrbitCamera.h"
#include "fpalg.h"
#define _USE_MATH_DEFINES
#include <math.h>

void OrbitCamera::CalcView(int w, int h, int x, int y)
{
    using fpalg::operator*;

    // view transform
    auto q_yaw = fpalg::QuaternionAxisAngle({0, 1, 0}, yawRadians);
    auto q_pitch = fpalg::QuaternionAxisAngle({1, 0, 0}, pitchRadians);
    auto transform = fpalg::Transform{shift, fpalg::QuaternionMul(q_pitch, q_yaw)};
    state.view = transform.Matrix();

    // inverse view transform
    auto inv = transform.Inverse();
    {
        state.rotation = inv.rotation;
        state.position = inv.position;
    }

    // ray for mouse cursor
    auto t = std::tan(state.fovYRadians / 2);
    const float xx = 2 * (float)x / w - 1;
    const float yy = 1 - 2 * (float)y / h;
    auto dir = fpalg::float3{
        t * aspectRatio * xx,
        t * yy,
        -1,
    };
    state.ray_direction = inv.ApplyDirection(dir);
    state.ray_origin = state.position;
}

void OrbitCamera::CalcPerspective()
{
    switch (perspectiveType)
    {
    case PerspectiveTypes::OpenGL:
        fpalg::PerspectiveRHGL(state.projection.data(), state.fovYRadians, aspectRatio, zNear, zFar);
        break;

    case PerspectiveTypes::D3D:
        fpalg::PerspectiveRHDX(state.projection.data(), state.fovYRadians, aspectRatio, zNear, zFar);
        break;
    }
}

void OrbitCamera::SetViewport(int x, int y, int w, int h)
{
    if (w == state.viewportWidth && h == state.viewportHeight)
    {
        return;
    }
    if (h == 0 || w == 0)
    {
        aspectRatio = 1.0f;
    }
    else
    {
        aspectRatio = w / (float)h;
    }
    state.viewportX = x;
    state.viewportY = y;
    state.viewportWidth = w;
    state.viewportHeight = h;
    CalcPerspective();
}

void OrbitCamera::WindowInput(const screenstate::ScreenState &window)
{
    SetViewport(0, 0, window.Width, window.Height);

    if (prevMouseX != -1 && prevMouseY != -1)
    {
        auto deltaX = window.MouseX - prevMouseX;
        auto deltaY = window.MouseY - prevMouseY;

        if (window.Has(screenstate::MouseButtonFlags::RightDown))
        {
            const auto FACTOR = 1.0f / 180.0f * 1.7f;
            yawRadians += deltaX * FACTOR;
            pitchRadians += deltaY * FACTOR;
        }
        if (window.Has(screenstate::MouseButtonFlags::MiddleDown))
        {
            shift[0] -= deltaX / (float)state.viewportHeight * shift[2];
            shift[1] += deltaY / (float)state.viewportHeight * shift[2];
        }
        if (window.Has(screenstate::WheelPlus))
        {
            shift[2] *= 0.9f;
        }
        else if (window.Has(screenstate::WheelMinus))
        {
            shift[2] *= 1.1f;
        }
    }
    prevMouseX = window.MouseX;
    prevMouseY = window.MouseY;
    CalcView(window.Width, window.Height, window.MouseX, window.MouseY);
    // state.viewProjection = state.view *  CalcViewProjection();
}


    // // mult
    // std::array<float, 16> viewProjection;
    // void CalcViewProjection()
    // {
    //     using fpalg::operator*;
    //     viewProjection = view * projection;
    // }

    // std::array<float, 16> CalcModelViewProjection(const std::array<float, 16> &m) const
    // {
    //     using fpalg::operator*;
    //     return m * viewProjection;
    // }
