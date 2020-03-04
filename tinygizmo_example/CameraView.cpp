#include "CameraView.h"
#include <tinygizmo.h>
#include <castalg.h>

void CameraView::update(float dt, int mouseX, int mouseY,
                        bool mouseRightDown, bool mouseMiddleDown, int mouseWheel)
{
    if (m_lastMouseX >= 0)
    {

        auto dx = mouseX - m_lastMouseX;
        auto dy = mouseY - m_lastMouseY;
        if (mouseRightDown)
        {
            yaw += (dx * dt);
            pitch += (dy * dt);
        }
        if (mouseMiddleDown)
        {
            shift[0] += dx * dt;
            shift[1] -= dy * dt;
        }
        if (mouseWheel)
        {
            // dolly
            if (mouseWheel > 0)
            {
                shift[2] *= 0.9f;
            }
            else
            {
                shift[2] *= 1.1f;
            }
        }
    }
    m_lastMouseX = mouseX;
    m_lastMouseY = mouseY;
    // lastState = state;

    // view transform
    auto q_yaw = castalg::quaternion::axisAngle(castalg::float3(0, 1, 0), yaw);
    auto q_pitch = castalg::quaternion::axisAngle(castalg::float3(1, 0, 0), pitch);
    auto transform = castalg::transform{shift, q_pitch * q_yaw};
    matrix = castalg::ref_cast<std::array<float, 16>>(transform.matrix());

    // inverse view transform
    {
        auto inv = transform.inverse();
        orientation = castalg::ref_cast<std::array<float, 4>>(inv.rotation);
        position = castalg::ref_cast<std::array<float, 3>>(inv.position);
    }
}

void CameraProjection::update(float aspectRatio)
{
    matrix = castalg::ref_cast<std::array<float, 16>>(castalg::matrix::perspectiveGLRH(yfov, aspectRatio, near_clip, far_clip));
}
