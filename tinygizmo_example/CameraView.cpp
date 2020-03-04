#include "CameraView.h"
#include <tinygizmo.h>
#include <castalg.h>


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

    // const auto orientation = get_orientation(yaw, pitch);
    auto q_yaw = castalg::quaternion::axisAngle(castalg::float3(0, 1, 0), yaw);
    auto q_pitch = castalg::quaternion::axisAngle(castalg::float3(1, 0, 0), pitch);
    auto orientation =  q_yaw * q_pitch;

    this->orientation = castalg::ref_cast<std::array<float, 4>>(orientation);
    auto inv_t = castalg::matrix::translation(-castalg::ref_float3(shift));
    auto inv_r = castalg::matrix::rotation(orientation.conjugate());
    matrix = (inv_r * inv_t).values;

    auto inv = orientation.rotate(shift);
    position[0] = inv.x;
    position[1] = inv.y;
    position[2] = inv.z;
}

#include <linalg.h>

void CameraProjection::update(float aspectRatio)
{
    matrix = castalg::ref_cast<std::array<float, 16>>(linalg::perspective_matrix(
        yfov, aspectRatio, near_clip, far_clip));
}
