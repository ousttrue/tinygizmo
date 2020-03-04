#include "orbit_camera.h"
#include "castalg.h"
#define _USE_MATH_DEFINES
#include <math.h>

void OrbitCamera::CalcView()
{
    using fpalg::operator*;

    // auto yaw = fpalg::YawMatrix(yawRadians);
    // auto pitch = fpalg::PitchMatrix(pitchRadians);
    // auto yawPitch = yaw * pitch;
    // auto t = fpalg::TranslationMatrix(-shiftX, -shiftY, -shiftZ);
    // state.view = yawPitch * t;

    // t[12] *= -1;
    // t[13] *= -1;
    // t[14] *= -1;
    // fpalg::Transpose(yawPitch);
    // state.viewInverse = t * yawPitch;

    // view transform
    auto q_yaw = castalg::quaternion::axisAngle(castalg::float3(0, 1, 0), yawRadians);
    auto q_pitch = castalg::quaternion::axisAngle(castalg::float3(1, 0, 0), pitchRadians);
    auto transform = castalg::transform{shift, q_pitch * q_yaw};
    state.view = castalg::ref_cast<std::array<float, 16>>(transform.matrix());

    // inverse view transform
    {
        auto inv = transform.inverse();
        state.rotation = castalg::ref_cast<std::array<float, 4>>(inv.rotation);
        state.position = castalg::ref_cast<std::array<float, 3>>(inv.position);
    }

}

void OrbitCamera::CalcPerspective()
{
#if 1
    fpalg::PerspectiveRHGL(state.projection.data(), state.fovYRadians, aspectRatio, zNear, zFar);
#else
    fpalg::PerspectiveRHDX(state.projection.data(), state.fovYRadians, aspectRatio, zNear, zFar);
#endif
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
    CalcView();
    state.CalcViewProjection();
}
