#include "CameraView.h"
#include <tinygizmo.h>
#include <linalg.h>
#include <castalg.h>

static castalg::matrix get_view_matrix(
    const castalg::quaternion &rotation, const std::array<float, 3> &position)
{
    auto inv_t = castalg::matrix::translation(-castalg::ref_float3(position));
    auto inv_r = castalg::matrix::rotation(rotation.conjugate());
    return inv_r * inv_t;
}

static castalg::quaternion get_orientation(float yaw, float pitch)
{
    auto q_yaw = castalg::quaternion::axisAngle(castalg::float3(0, 1, 0), yaw);
    auto q_pitch = castalg::quaternion::axisAngle(castalg::float3(1, 0, 0), pitch);
    return q_yaw * q_pitch;
}

void CameraView::update(struct WindowState &state)
{
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

    const auto orientation = get_orientation(yaw, pitch);
    this->orientation = castalg::ref_cast<std::array<float, 4>>(orientation);
    matrix = get_view_matrix(orientation, shift).values;

    auto inv = linalg::inverse(castalg::ref_cast<linalg::aliases::float4x4>(matrix));
    position[0] = inv.w.x;
    position[1] = inv.w.y;
    position[2] = inv.w.z;
}

void CameraProjection::update(float aspectRatio)
{
    matrix = castalg::ref_cast<std::array<float, 16>>(linalg::perspective_matrix(
        yfov, aspectRatio, near_clip, far_clip));
}
