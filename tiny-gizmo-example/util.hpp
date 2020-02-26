// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org>

#pragma once

#ifndef tinygizmo_example_util_hpp
#define tinygizmo_example_util_hpp

#include <functional>
#include <vector>

#define GLEW_STATIC
#define GL_GLEXT_PROTOTYPES
#include "glew.h"

#define GLFW_INCLUDE_GLU
#include "GLFW\glfw3.h"

#include <tiny-gizmo.hpp>
#include "linalg.h"

///////////////////////////////////
//   Windowing & App Lifecycle   //
///////////////////////////////////

struct ray { linalg::aliases::float3 origin; linalg::aliases::float3 direction; };

struct rect
{
    int x0, y0, x1, y1;
    int width() const { return x1 - x0; }
    int height() const { return y1 - y0; }
    linalg::aliases::int2 dims() const { return{ width(), height() }; }
    float aspect_ratio() const { return (float)width() / height(); }
};

struct camera
{
    float yfov, near_clip, far_clip;
    linalg::aliases::float3 position;
    float pitch, yaw;
    linalg::aliases::float4 get_orientation() const { return qmul(rotation_quat(linalg::aliases::float3(0, 1, 0), yaw), rotation_quat(linalg::aliases::float3(1, 0, 0), pitch)); }
    linalg::aliases::float4x4 get_view_matrix() const { return mul(rotation_matrix(qconj(get_orientation())), translation_matrix(-position)); }
    linalg::aliases::float4x4 get_projection_matrix(const float aspectRatio) const { return linalg::perspective_matrix(yfov, aspectRatio, near_clip, far_clip); }
    linalg::aliases::float4x4 get_viewproj_matrix(const float aspectRatio) const { return mul(get_projection_matrix(aspectRatio), get_view_matrix()); }
};

// Returns a world-space ray through the given pixel, originating at the camera
ray get_ray_from_pixel(const linalg::aliases::float2 & pixel, const rect & viewport, const camera & cam)
{
    const float x = 2 * (pixel.x - viewport.x0) / viewport.width() - 1, y = 1 - 2 * (pixel.y - viewport.y0) / viewport.height();
    const linalg::aliases::float4x4 inv_view_proj = inverse(cam.get_viewproj_matrix(viewport.aspect_ratio()));
    const linalg::aliases::float4 p0 = mul(inv_view_proj, linalg::aliases::float4(x, y, -1, 1)), p1 = mul(inv_view_proj, linalg::aliases::float4(x, y, +1, 1));
    return{ cam.position, p1.xyz()*p0.w - p0.xyz()*p1.w };
}


#endif // end tinygizmo_example_util_hpp
