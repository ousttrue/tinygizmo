#include "CameraView.h"
#include <tiny-gizmo.hpp>
#include <linalg.h>
#include <castalg.h>

static linalg::aliases::float4x4 get_view_matrix(
    const linalg::aliases::float3 &position,
    const linalg::aliases::float4 &orientation)
{
    return mul(
        rotation_matrix(qconj(orientation)),
        translation_matrix(-position));
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
    else if (state.mouseRightDown)
    {
        linalg::aliases::float3 move;
        if (state.keycode['W'])
            move -= qzdir(orientation);
        if (state.keycode['A'])
            move -= qxdir(orientation);
        if (state.keycode['S'])
            move += qzdir(orientation);
        if (state.keycode['D'])
            move += qxdir(orientation);
        float timestep = std::chrono::duration<float>(state.time - lastState.time).count();

        if (length2(move) > 0)
        {
            move = normalize(move) * (timestep * 10);
            position[0] += move.x;
            position[1] += move.y;
            position[2] += move.z;
        }

        yaw -= (state.mouseX - lastState.mouseX) * 0.01f;
        pitch -= (state.mouseY - lastState.mouseY) * 0.01f;
    }
    lastState = state;

    matrix = castalg::ref_cast<std::array<float, 16>>(get_view_matrix(
        castalg::ref_cast<linalg::aliases::float3>(position),
        castalg::ref_cast<linalg::aliases::float4>(get_orientation(yaw, pitch))));

    this->orientation = castalg::ref_cast<std::array<float, 4>>(orientation);
}

void CameraProjection::update(float yfov, float aspectRatio, float near_clip, float far_clip)
{
    matrix = castalg::ref_cast<std::array<float, 16>>(linalg::perspective_matrix(
        yfov, aspectRatio, near_clip, far_clip));
}
