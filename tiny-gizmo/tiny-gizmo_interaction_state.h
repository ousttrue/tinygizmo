#pragma once
#include "minalg.h"

namespace tinygizmo
{
struct gizmo_application_state;
struct ray;
struct gizmo_mesh_component;

struct interaction_state
{
    bool active{false};                   // Flag to indicate if the gizmo is being actively manipulated
    bool hover{false};                    // Flag to indicate if the gizmo is being hovered
    minalg::float3 original_position;     // Original position of an object being manipulated with a gizmo
    minalg::float4 original_orientation;  // Original orientation of an object being manipulated with a gizmo
    minalg::float3 original_scale;        // Original scale of an object being manipulated with a gizmo
    minalg::float3 click_offset;          // Offset from position of grabbed object to coordinates of clicked point
    gizmo_mesh_component *mesh = nullptr; // Currently active component
    minalg::float3 axis;

    ///////////////////////////////////
    // Private Gizmo Implementations //
    ///////////////////////////////////
    void axis_translation_dragger(const gizmo_application_state &state, const ray &ray, const minalg::float3 &axis, minalg::float3 &point);
    void plane_translation_dragger(const gizmo_application_state &state, const ray &ray, const minalg::float3 &plane_normal, minalg::float3 &point);

    minalg::float4 axis_rotation_dragger(const gizmo_application_state &state, const ray &ray,
                                         const minalg::float3 &axis, const minalg::float3 &center, const minalg::float4 &start_orientation);

    void axis_scale_dragger(const gizmo_application_state &state, const ray &ray,
                            const minalg::float3 &axis, const minalg::float3 &center, const bool uniform, minalg::float3 *scale);
};

} // namespace  tinygizmo
