#include "CameraView.h"
#include <tiny-gizmo.hpp>
#include <linalg.h>
#include <castalg.h>

static std::array<float, 16> get_view_matrix(
    const std::array<float, 4> &rotation, const std::array<float, 3> &position)
{
    auto t= translation_matrix(-castalg::ref_cast<linalg::aliases::float3>(position));
    auto r = rotation_matrix(qconj(castalg::ref_cast<linalg::aliases::float4>(rotation)));
    return castalg::ref_cast<std::array<float, 16>>(mul(t, r));
}

static std::array<float, 4> get_orientation(float yaw, float pitch)
{
    return castalg::ref_cast<std::array<float, 4>>(qmul(
        rotation_quat(linalg::aliases::float3(0, 1, 0), yaw),
        rotation_quat(linalg::aliases::float3(1, 0, 0), pitch)));
}

void CameraView::update(struct WindowState &state)
{
    const auto orientation = castalg::ref_cast<linalg::aliases::float4>(get_orientation(yaw, pitch));
    if (lastState.windowHeight == 0)
    {
        // skip
    }
    else
    {
        auto dx = state.mouseX - lastState.mouseX;
        auto dy = state.mouseY - lastState.mouseY;
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(state.time - lastState.time).count() * 0.001f;
        if (state.mouseRightDown)
        {
            yaw -= (dx * dt);
            pitch -= (dy * dt);
        }
        if (state.mouseMiddleDown)
        {
            shift[0] -= dx * dt;
            shift[1] += dy * dt;
        }
        if (state.mouseWheel)
        {
            // dolly
            if (state.mouseWheel > 0)
            {
                shift[2] *= 0.9f;
            }
            else
            {
                shift[2] *= 1.1f;
            }
        }
    }
    lastState = state;

    this->orientation = castalg::ref_cast<std::array<float, 4>>(orientation);
    matrix = get_view_matrix(this->orientation, shift);
    auto inv = linalg::inverse(castalg::ref_cast<linalg::aliases::float4x4>(matrix));
    position[0] = inv.w.x;
    position[1] = inv.w.y;
    position[2] = inv.w.z;
}

void CameraProjection::update(float yfov, float aspectRatio, float near_clip, float far_clip)
{
    matrix = castalg::ref_cast<std::array<float, 16>>(linalg::perspective_matrix(
        yfov, aspectRatio, near_clip, far_clip));
}
