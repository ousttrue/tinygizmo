#include "tinygizmo.h"
#include "utilmath.h"
#include "impl.h"
#include "../screenstate/castalg.h"

namespace tinygizmo
{
enum class interact
{
    none,
    scale_x,
    scale_y,
    scale_z,
    scale_xyz,
};

static const interact scale_components[] = {
    interact::scale_x,
    interact::scale_y,
    interact::scale_z,
};

static gizmo_mesh_component *get_mesh(interact c)
{
    static std::vector<minalg::float2> mace_points = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.1f}, {1.25f, 0.1f}, {1.25f, 0}};

    switch (c)
    {
    case interact::scale_x:
    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, mace_points),
            {1, 0.5f, 0.5f, 1.f},
            {1, 0, 0, 1.f},
            {1, 0, 0},
        };
        return &component;
    }
    case interact::scale_y:
    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, mace_points),
            {0.5f, 1, 0.5f, 1.f},
            {0, 1, 0, 1.f},
            {0, 1, 0},
        };
        return &component;
    }
    case interact::scale_z:
    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, mace_points),
            {0.5f, 0.5f, 1, 1.f},
            {0, 0, 1, 1.f},
            {0, 0, 1},
        };
        return &component;
    }
    }

    return nullptr;
}

void axis_scale_dragger(interaction_state &gizmo,
                        const gizmo_application_state &state, const ray &ray,
                        const minalg::float3 &axis, const minalg::float3 &center, const bool uniform,
                        minalg::float3 *scale)
{
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

        auto offset_on_axis = (distance - gizmo.click_offset) * axis;
        flush_to_zero(offset_on_axis);
        auto new_scale = gizmo.original_scale + offset_on_axis;

        if (uniform)
            *scale = minalg::float3(clamp(dot(distance, new_scale), 0.01f, 1000.f));
        else
            *scale = minalg::float3(clamp(new_scale.x, 0.01f, 1000.f), clamp(new_scale.y, 0.01f, 1000.f), clamp(new_scale.z, 0.01f, 1000.f));
        if (state.snap_scale)
            *scale = snap(*scale, state.snap_scale);
    }
}

static void dragger(interaction_state &gizmo, const gizmo_application_state &state, const ray &ray,
                    const minalg::float3 &center, minalg::float3 *scale, bool is_uniform)
{
    if (gizmo.active)
    {
        axis_scale_dragger(gizmo, state, ray, gizmo.mesh->axis, center, is_uniform, scale);
    }
}

bool scale_gizmo(const gizmo_system &ctx, const std::string &name, fpalg::TRS &trs, bool is_uniform)
{
    auto &t = castalg::ref_cast<rigid_transform>(trs);
    auto &impl = ctx.m_impl;
    auto &scale = t.scale;
    rigid_transform p = rigid_transform(t.orientation, t.position);
    const float draw_scale = impl->get_gizmo_scale(p.position);
    const uint32_t id = hash_fnv1a(name);
    auto &gizmo = impl->gizmos[id];

    // update
    interact updated_state = interact::none;
    auto ray = detransform(p, impl->get_ray());
    detransform(draw_scale, ray);
    float best_t = std::numeric_limits<float>::infinity();
    for (auto c : scale_components)
    {
        auto t = intersect_ray_mesh(ray, get_mesh(c)->mesh);
        if (t < best_t)
        {
            updated_state = c;
            best_t = t;
        }
    }
    if (impl->state.has_clicked)
    {
        gizmo.mesh = get_mesh(updated_state);
        if (gizmo.mesh)
        {
            transform(draw_scale, ray);
            gizmo.original_scale = scale;
            gizmo.click_offset = p.transform_point(ray.origin + ray.direction * best_t);
            gizmo.active = true;
        }
        else
            gizmo.active = false;
    }
    if (impl->state.has_released)
    {
        gizmo.mesh = nullptr;
        gizmo.active = false;
    }

    // drag
    dragger(gizmo, impl->state, impl->get_ray(), t.position, &scale, is_uniform);

    // draw
    auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());
    auto scaleMatrix = scaling_matrix(minalg::float3(draw_scale));
    modelMatrix = mul(modelMatrix, scaleMatrix);

    std::vector<interact> draw_components{interact::scale_x, interact::scale_y, interact::scale_z};

    for (auto c : draw_components)
    {
        auto mesh = get_mesh(c);
        gizmo_renderable r{
            .mesh = mesh->mesh,
            .color = (mesh == gizmo.mesh) ? mesh->base_color : mesh->highlight_color,
        };
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        impl->drawlist.push_back(r);
    }

    // return gizmo;
    return (gizmo.hover || gizmo.active);
}

} // namespace tinygizmo
