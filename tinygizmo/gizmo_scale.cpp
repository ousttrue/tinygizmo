#include "tinygizmo.h"
#include "utilmath.h"
#include "impl.h"
#include "../screenstate/castalg.h"
#include <iostream>

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

bool scale_gizmo(const gizmo_system &ctx, const std::string &name, fpalg::TRS &trs, bool is_uniform)
{
    auto &t = castalg::ref_cast<rigid_transform>(trs);
    auto &impl = ctx.m_impl;
    auto &scale = t.scale;
    rigid_transform p = rigid_transform(t.orientation, t.position);
    const uint32_t id = hash_fnv1a(name);
    auto &gizmo = impl->gizmos[id];

    // update
    interact updated_state = interact::none;
    auto ray = detransform(p, impl->get_ray());
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
            gizmo.original_scale = scale;
            gizmo.click_offset = p.transform_point(ray.origin + ray.direction * best_t);
            std::cout << best_t << gizmo.click_offset << std::endl;
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
    if (gizmo.active)
    {
        gizmo.axis_scale_dragger(state, ray, center, is_uniform, scale);
    }

    // draw
    auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());

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
