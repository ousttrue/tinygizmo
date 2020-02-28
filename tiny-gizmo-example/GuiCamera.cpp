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

std::array<float, 4> GuiCamera::get_orientation() const
{
    return castalg::ref_cast<std::array<float, 4>>(qmul(
        rotation_quat(linalg::aliases::float3(0, 1, 0), yaw),
        rotation_quat(linalg::aliases::float3(1, 0, 0), pitch)));
}

std::array<float, 16> GuiCamera::get_viewproj_matrix(const float aspectRatio) const
{
    auto m = mul(get_projection_matrix(aspectRatio, params),
                 get_view_matrix(
                     castalg::ref_cast<linalg::aliases::float3>(params.position),
                     castalg::ref_cast<linalg::aliases::float4>(get_orientation())));
    return castalg::ref_cast<std::array<float, 16>>(m);
}

// Returns a world-space ray through the given pixel, originating at the camera
std::array<float, 3> GuiCamera::get_ray_direction(int _x, int _y, int w, int h)
{
    const float x = 2 * (float)_x / w - 1;
    const float y = 1 - 2 * (float)_y / h;
    auto aspect_ratio = w / (float)h;
    const linalg::aliases::float4x4 inv_view_proj = inverse(castalg::ref_cast<linalg::aliases::float4x4>(this->get_viewproj_matrix(aspect_ratio)));
    const linalg::aliases::float4 p0 = mul(inv_view_proj, linalg::aliases::float4(x, y, -1, 1)), p1 = mul(inv_view_proj, linalg::aliases::float4(x, y, +1, 1));
    return castalg::ref_cast<std::array<float, 3>>(p1.xyz() * p0.w - p0.xyz() * p1.w);
}
