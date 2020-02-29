#pragma once
#include "minalg.h"

namespace tinygizmo
{
struct gizmo_application_state;
struct ray;

enum class interact
{
    none,
    translate_x,
    translate_y,
    translate_z,
    translate_yz,
    translate_zx,
    translate_xy,
    translate_xyz,
    rotate_x,
    rotate_y,
    rotate_z,
    scale_x,
    scale_y,
    scale_z,
    scale_xyz,
};

struct interaction_state
{
    bool active{false};                  // Flag to indicate if the gizmo is being actively manipulated
    bool hover{false};                   // Flag to indicate if the gizmo is being hovered
    minalg::float3 original_position;    // Original position of an object being manipulated with a gizmo
    minalg::float4 original_orientation; // Original orientation of an object being manipulated with a gizmo
    minalg::float3 original_scale;       // Original scale of an object being manipulated with a gizmo
    minalg::float3 click_offset;         // Offset from position of grabbed object to coordinates of clicked point
    interact interaction_mode;           // Currently active component

    ///////////////////////////////////
    // Private Gizmo Implementations //
    ///////////////////////////////////

    void axis_translation_dragger(const gizmo_application_state &state, const ray &ray, const minalg::float3 &axis, minalg::float3 &point);
    void plane_translation_dragger(const gizmo_application_state &state, const ray &ray, const minalg::float3 &plane_normal, minalg::float3 &point);

    minalg::float4 rotation_dragger(const gizmo_application_state &state, const ray &ray,
                                    const minalg::float3 &center, bool is_local)
    {
        auto starting_orientation = is_local ? original_orientation : minalg::float4(0, 0, 0, 1);
        switch (interaction_mode)
        {
        case interact::rotate_x:
            return axis_rotation_dragger(state, ray, {1, 0, 0}, center, starting_orientation);
        case interact::rotate_y:
            return axis_rotation_dragger(state, ray, {0, 1, 0}, center, starting_orientation);
        case interact::rotate_z:
            return axis_rotation_dragger(state, ray, {0, 0, 1}, center, starting_orientation);
        }
        throw;
    }
    minalg::float4 axis_rotation_dragger(const gizmo_application_state &state, const ray &ray,
                                         const minalg::float3 &axis, const minalg::float3 &center, const minalg::float4 &start_orientation);

    void scale_dragger(const gizmo_application_state &state, const ray &ray,
                       const minalg::float3 &center, minalg::float3 *scale, bool isUniform)
    {
        if (active)
        {
            switch (interaction_mode)
            {
            case interact::scale_x:
                axis_scale_dragger(state, ray, {1, 0, 0}, center, isUniform, scale);
                break;
            case interact::scale_y:
                axis_scale_dragger(state, ray, {0, 1, 0}, center, isUniform, scale);
                break;
            case interact::scale_z:
                axis_scale_dragger(state, ray, {0, 0, 1}, center, isUniform, scale);
                break;
            }
        }
    }
    void axis_scale_dragger(const gizmo_application_state &state, const ray &ray,
                            const minalg::float3 &axis, const minalg::float3 &center, const bool uniform, minalg::float3 *scale);
};

} // namespace  tinygizmo