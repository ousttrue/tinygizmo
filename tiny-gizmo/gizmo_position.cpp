#include "tiny-gizmo.hpp"
#include "tiny-gizmo_interaction_state.h"
#include "utilmath.h"
#include "minalg.h"
#include "impl.h"

namespace tinygizmo
{
static const interact translation_components[] = {
    interact::translate_x,
    interact::translate_y,
    interact::translate_z,
    interact::translate_xy,
    interact::translate_yz,
    interact::translate_zx,
    interact::translate_xyz,
};

static gizmo_mesh_component &get_mesh(interact component)
{
    static std::vector<minalg::float2> arrow_points = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.10f}, {1.2f, 0}};

    switch (component)
    {
    case interact::translate_x:
    {
        static gizmo_mesh_component component{geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, arrow_points), {1, 0.5f, 0.5f, 1.f}, {1, 0, 0, 1.f}};
        return component;
    }
    case interact::translate_y:
    {
        static gizmo_mesh_component component{geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, arrow_points), {0.5f, 1, 0.5f, 1.f}, {0, 1, 0, 1.f}};
        return component;
    }
    case interact::translate_z:
    {
        static gizmo_mesh_component component{geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, arrow_points), {0.5f, 0.5f, 1, 1.f}, {0, 0, 1, 1.f}};
        return component;
    }
    case interact::translate_xy:
    {
        static gizmo_mesh_component component{geometry_mesh::make_box_geometry({0.25, 0.25, -0.01f}, {0.75f, 0.75f, 0.01f}), {1, 1, 0.5f, 0.5f}, {1, 1, 0, 0.6f}};
        return component;
    }
    case interact::translate_yz:
    {
        static gizmo_mesh_component component{geometry_mesh::make_box_geometry({-0.01f, 0.25, 0.25}, {0.01f, 0.75f, 0.75f}), {0.5f, 1, 1, 0.5f}, {0, 1, 1, 0.6f}};
        return component;
    }
    case interact::translate_zx:
    {
        static gizmo_mesh_component component{geometry_mesh::make_box_geometry({0.25, -0.01f, 0.25}, {0.75f, 0.01f, 0.75f}), {1, 0.5f, 1, 0.5f}, {1, 0, 1, 0.6f}};
        return component;
    }
    case interact::translate_xyz:
    {
        static gizmo_mesh_component component{geometry_mesh::make_box_geometry({-0.05f, -0.05f, -0.05f}, {0.05f, 0.05f, 0.05f}), {0.9f, 0.9f, 0.9f, 0.25f}, {1, 1, 1, 0.35f}};
        return component;
    }
    }

    throw;
}

// check hit
void raycast(gizmo_context_impl *impl, interaction_state *self, const rigid_transform &p, float draw_scale, bool is_local)
{
    interact updated_state = interact::none;
    auto ray = detransform(p, impl->get_ray());
    detransform(draw_scale, ray);

    float best_t = std::numeric_limits<float>::infinity();
    for (auto c : translation_components)
    {
        auto t = intersect_ray_mesh(ray, get_mesh(c).mesh);
        if (t < best_t)
        {
            updated_state = c;
            best_t = t;
        }
    }

    if (impl->state.has_clicked)
    {
        self->interaction_mode = updated_state;

        if (self->interaction_mode != interact::none)
        {
            transform(draw_scale, ray);
            self->click_offset = is_local ? p.transform_vector(ray.origin + ray.direction * best_t) : ray.origin + ray.direction * best_t;
            self->active = true;
        }
        else
        {
            self->active = false;
        }
    }

    self->hover = (best_t == std::numeric_limits<float>::infinity()) ? false : true;

    if (impl->state.has_released)
    {
        self->interaction_mode = interact::none;
        self->active = false;
    }
}

void draw(gizmo_context_impl *impl, const rigid_transform &p, float draw_scale, interact mode)
{
    auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());
    auto scaleMatrix = scaling_matrix(minalg::float3(draw_scale));
    modelMatrix = mul(modelMatrix, scaleMatrix);

    for (auto c : translation_components)
    {
        auto &mesh = get_mesh(c);
        gizmo_renderable r{
            .mesh = mesh.mesh,
            .color = (c == mode) ? mesh.base_color : mesh.highlight_color,
        };
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        impl->drawlist.push_back(r);
    }
}

bool position_gizmo(const gizmo_context &ctx, const std::string &name, rigid_transform &t, bool is_local)
{
    auto &impl = ctx.m_impl;
    auto self = &impl->gizmos[hash_fnv1a(name)];
    auto p = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position);
    const float draw_scale = impl->get_gizmo_scale(t.position);

    // update
    raycast(impl, self, p, draw_scale, is_local);

    // drag
    if (self->active)
    {
        minalg::float3 local_axes[3]{qxdir(p.orientation), qydir(p.orientation), qzdir(p.orientation)};
        static minalg::float3 world_axes[3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        self->translation_dragger(impl->state, impl->get_ray(), is_local ? local_axes : world_axes, t.position);
    }

    // draw
    draw(impl, p, draw_scale, self->interaction_mode);

    return (self->hover || self->active);
}

} // namespace tinygizmo
