#include "tiny-gizmo.hpp"
#include "tiny-gizmo_interaction_state.h"
#include "utilmath.h"
#include "impl.h"
#include "assert.h"
#include "minalg.h"
#include "rigid_transform.h"

namespace tinygizmo
{

static const interact orientation_components[] = {
    interact::rotate_x,
    interact::rotate_y,
    interact::rotate_z,
};

static gizmo_mesh_component &get_mesh(interact c)
{
    static std::vector<minalg::float2> ring_points = {{+0.025f, 1}, {-0.025f, 1}, {-0.025f, 1}, {-0.025f, 1.1f}, {-0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1}};

    switch (c)
    {
    case interact::rotate_x:
    {
        static gizmo_mesh_component component{geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 32, ring_points, 0.003f), {1, 0.5f, 0.5f, 1.f}, {1, 0, 0, 1.f}};
        return component;
    }
    case interact::rotate_y:
    {
        static gizmo_mesh_component component{geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 32, ring_points, -0.003f), {0.5f, 1, 0.5f, 1.f}, {0, 1, 0, 1.f}};
        return component;
    }
    case interact::rotate_z:
    {
        static gizmo_mesh_component component{geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 32, ring_points), {0.5f, 0.5f, 1, 1.f}, {0, 0, 1, 1.f}};
        return component;
    }
    }

    throw;
}

bool orientation_gizmo(const gizmo_context &ctx, const std::string &name, rigid_transform &t, bool is_local)
{
    auto &impl = ctx.m_impl;
    assert(length2(t.orientation) > float(1e-6));
    rigid_transform p = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position); // Orientation is local by default
    const float draw_scale = impl->get_gizmo_scale(p.position);
    const uint32_t id = hash_fnv1a(name);
    auto &gizmo = impl->gizmos[id];

    // update
    interact updated_state = interact::none;
    auto ray = detransform(p, impl->get_ray());
    detransform(draw_scale, ray);
    float best_t = std::numeric_limits<float>::infinity();
    for (auto c : orientation_components)
    {
        auto f = intersect_ray_mesh(ray, get_mesh(c).mesh);
        if (f < best_t)
        {
            updated_state = c;
            best_t = f;
        }
    }
    if (impl->state.has_clicked)
    {
        gizmo.interaction_mode = updated_state;
        if (gizmo.interaction_mode != interact::none)
        {
            transform(draw_scale, ray);
            gizmo.original_position = t.position;
            gizmo.original_orientation = t.orientation;
            gizmo.click_offset = p.transform_point(ray.origin + ray.direction * best_t);
            gizmo.active = true;
        }
        else
            gizmo.active = false;
    }
    if (impl->state.has_released)
    {
        gizmo.interaction_mode = interact::none;
        gizmo.active = false;
    }

    // drag
    if (gizmo.active)
    {
        p.orientation = gizmo.rotation_dragger(impl->state, impl->get_ray(), t.position, is_local);
    }

    // draw
    auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());
    auto scaleMatrix = scaling_matrix(minalg::float3(draw_scale));
    modelMatrix = mul(modelMatrix, scaleMatrix);

    std::vector<interact> draw_interactions;
    if (!is_local && gizmo.interaction_mode != interact::none)
        draw_interactions = {gizmo.interaction_mode};
    else
        draw_interactions = {interact::rotate_x, interact::rotate_y, interact::rotate_z};

    for (auto c : draw_interactions)
    {
        auto &mesh = get_mesh(c);
        gizmo_renderable r{
            .mesh = mesh.mesh,
            .color = (c == gizmo.interaction_mode) ? mesh.base_color : mesh.highlight_color,
        };
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        impl->drawlist.push_back(r);
    }

    // For non-local transformations, we only present one rotation ring
    // and draw an arrow from the center of the gizmo to indicate the degree of rotation
    if (!is_local && gizmo.interaction_mode != interact::none)
    {
        minalg::float3 activeAxis;
        switch (gizmo.interaction_mode)
        {
        case interact::rotate_x:
            activeAxis = {1, 0, 0};
            break;
        case interact::rotate_y:
            activeAxis = {0, 1, 0};
            break;
        case interact::rotate_z:
            activeAxis = {0, 0, 1};
            break;
        }

        interaction_state &interaction = gizmo;

        // Create orthonormal basis for drawing the arrow
        auto a = qrot(p.orientation, interaction.click_offset - interaction.original_position);
        auto zDir = normalize(activeAxis), xDir = normalize(cross(a, zDir)), yDir = cross(zDir, xDir);

        // Ad-hoc geometry
        std::initializer_list<minalg::float2> arrow_points = {{0.0f, 0.f}, {0.0f, 0.05f}, {0.8f, 0.05f}, {0.9f, 0.10f}, {1.0f, 0}};
        auto geo = geometry_mesh::make_lathed_geometry(yDir, xDir, zDir, 32, arrow_points);

        gizmo_renderable r;
        r.mesh = geo;
        r.color = minalg::float4(1);
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position);
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        impl->drawlist.push_back(r);

        t.orientation = qmul(p.orientation, interaction.original_orientation);
    }
    else if (is_local == true && gizmo.interaction_mode != interact::none)
        t.orientation = p.orientation;

    return (gizmo.hover || gizmo.active);
}

} // namespace tinygizmo
