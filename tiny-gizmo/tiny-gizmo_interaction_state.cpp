#include "minalg.h"
#include "tiny-gizmo.hpp"
#include "rigid_transform.h"
#include "utilmath.h"
#include "tiny-gizmo_interaction_state.h"
#include <castalg.h>

namespace tinygizmo
{

void interaction_state::axis_translation_dragger(const gizmo_application_state &state, const ray &ray, const minalg::float3 &axis, minalg::float3 &point)
{
    if (state.mouse_left)
    {
        // First apply a plane translation dragger with a plane that contains the desired axis and is oriented to face the camera
        auto plane_tangent = minalg::cross(axis, point - castalg::ref_cast<minalg::float3>(state.camera_position));
        auto plane_normal = cross(axis, plane_tangent);
        this->plane_translation_dragger(state, ray, plane_normal, point);

        // Constrain object motion to be along the desired axis
        point = this->original_position + axis * dot(point - this->original_position, axis);
    }
}

void interaction_state::plane_translation_dragger(const gizmo_application_state &state, const ray &r, const minalg::float3 &plane_normal, minalg::float3 &point)
{
    // Mouse clicked
    if (state.has_clicked)
        this->original_position = point;

    if (state.mouse_left)
    {
        // Define the plane to contain the original position of the object
        auto plane_point = this->original_position;

        // If an intersection exists between the ray and the plane, place the object at that point
        const float denom = dot(r.direction, plane_normal);
        if (std::abs(denom) == 0)
            return;

        const float t = dot(plane_point - r.origin, plane_normal) / denom;
        if (t < 0)
            return;

        point = r.origin + r.direction * t;

        if (state.snap_translation)
            point = snap(point, state.snap_translation);
    }
}

minalg::float4 interaction_state::axis_rotation_dragger(const gizmo_application_state &state, const ray &r,
                                                        const minalg::float3 &axis, const minalg::float3 &center, const minalg::float4 &start_orientation)
{
    if (state.mouse_left)
    {
        rigid_transform original_pose = {start_orientation, original_position};
        auto the_axis = original_pose.transform_vector(axis);
        minalg::float4 the_plane = {the_axis, -dot(the_axis, click_offset)};

        float t;
        if (intersect_ray_plane(r, the_plane, &t))
        {
            auto center_of_rotation = original_position + the_axis * dot(the_axis, click_offset - original_position);
            auto arm1 = normalize(click_offset - center_of_rotation);
            auto arm2 = normalize(r.origin + r.direction * t - center_of_rotation);

            float d = dot(arm1, arm2);
            if (d > 0.999f)
            {
                return start_orientation;
            }

            float angle = std::acos(d);
            if (angle < 0.001f)
            {
                return start_orientation;
            }

            if (state.snap_rotation)
            {
                auto snapped = make_rotation_quat_between_vectors_snapped(arm1, arm2, state.snap_rotation);
                return qmul(snapped, start_orientation);
            }
            else
            {
                auto a = normalize(cross(arm1, arm2));
                return qmul(rotation_quat(a, angle), start_orientation);
            }
        }
    }
}

void interaction_state::axis_scale_dragger(const gizmo_application_state &state, const ray &ray,
                                           const minalg::float3 &axis, const minalg::float3 &center, const bool uniform,
                                           minalg::float3 *scale)
{
    // interaction_state &interaction = g.gizmos[id];

    if (state.mouse_left)
    {
        auto plane_tangent = cross(axis, center - castalg::ref_cast<minalg::float3>(state.camera_position));
        auto plane_normal = cross(axis, plane_tangent);

        minalg::float3 distance;
        if (state.mouse_left)
        {
            // Define the plane to contain the original position of the object
            auto plane_point = center;

            // If an intersection exists between the ray and the plane, place the object at that point
            const float denom = dot(ray.direction, plane_normal);
            if (std::abs(denom) == 0)
                return;

            const float t = dot(plane_point - ray.origin, plane_normal) / denom;
            if (t < 0)
                return;

            distance = ray.origin + ray.direction * t;
        }

        auto offset_on_axis = (distance - this->click_offset) * axis;
        flush_to_zero(offset_on_axis);
        auto new_scale = this->original_scale + offset_on_axis;

        if (uniform)
            *scale = minalg::float3(clamp(dot(distance, new_scale), 0.01f, 1000.f));
        else
            *scale = minalg::float3(clamp(new_scale.x, 0.01f, 1000.f), clamp(new_scale.y, 0.01f, 1000.f), clamp(new_scale.z, 0.01f, 1000.f));
        if (state.snap_scale)
            *scale = snap(*scale, state.snap_scale);
    }
}

} // namespace tinygizmo
