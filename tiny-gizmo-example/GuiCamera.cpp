#include "GuiCamera.h"
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

static linalg::aliases::float4x4 get_projection_matrix(const float aspectRatio, const tinygizmo::camera_parameters &params)
{
    return linalg::perspective_matrix(params.yfov, aspectRatio, params.near_clip, params.far_clip);
}

static std::array<float, 4> get_orientation(float yaw, float pitch)
{
    return castalg::ref_cast<std::array<float, 4>>(qmul(
        rotation_quat(linalg::aliases::float3(0, 1, 0), yaw),
        rotation_quat(linalg::aliases::float3(1, 0, 0), pitch)));
}

static std::array<float, 16> get_viewproj_matrix(const float aspectRatio, const tinygizmo::camera_parameters &params, float yaw, float pitch)
{
    auto m = mul(get_projection_matrix(aspectRatio, params),
                 get_view_matrix(
                     castalg::ref_cast<linalg::aliases::float3>(params.position),
                     castalg::ref_cast<linalg::aliases::float4>(get_orientation(yaw, pitch))));
    return castalg::ref_cast<std::array<float, 16>>(m);
}

// Returns a world-space ray through the given pixel, originating at the camera
static std::array<float, 3> get_ray_direction(int _x, int _y, int w, int h, const linalg::aliases::float4x4 &viewProjMatrix)
{
    const float x = 2 * (float)_x / w - 1;
    const float y = 1 - 2 * (float)_y / h;
    auto aspect_ratio = w / (float)h;
    const linalg::aliases::float4x4 inv_view_proj = inverse(viewProjMatrix);
    const linalg::aliases::float4 p0 = mul(inv_view_proj, linalg::aliases::float4(x, y, -1, 1)), p1 = mul(inv_view_proj, linalg::aliases::float4(x, y, +1, 1));
    return castalg::ref_cast<std::array<float, 3>>(p1.xyz() * p0.w - p0.xyz() * p1.w);
}

void GuiCamera::update(struct WindowState &state,
                       tinygizmo::camera_parameters &params)
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
            params.position[0] += move.x;
            params.position[1] += move.y;
            params.position[2] += move.z;
        }

        yaw -= (state.mouseX - lastState.mouseX) * 0.01f;
        pitch -= (state.mouseY - lastState.mouseY) * 0.01f;
    }
    lastState = state;

    view_proj_matrix = get_viewproj_matrix(state.aspectRatio(), params, yaw, pitch);

    params.orientation = castalg::ref_cast<std::array<float, 4>>(orientation);
    ray_dir = get_ray_direction(
        state.mouseX, state.mouseY, state.windowWidth, state.windowHeight,
        castalg::ref_cast<linalg::aliases::float4x4>(view_proj_matrix));
}
