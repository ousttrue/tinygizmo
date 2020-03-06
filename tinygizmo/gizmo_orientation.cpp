#include "tinygizmo.h"
#include "utilmath.h"
#include "impl.h"
#include "assert.h"
#include "minalg.h"
#include "rigid_transform.h"
#include "../screenstate/castalg.h"

namespace tinygizmo
{

enum class interact
{
    none,
    rotate_x,
    rotate_y,
    rotate_z,
};

static const interact orientation_components[] = {
    interact::rotate_x,
    interact::rotate_y,
    interact::rotate_z,
};

static gizmo_mesh_component *get_mesh(interact c)
{
    static minalg::float2 ring_points[] = {{+0.025f, 1}, {-0.025f, 1}, {-0.025f, 1}, {-0.025f, 1.1f}, {-0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1}};

    switch (c)
    {
    case interact::rotate_x:
    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 32, ring_points, _countof(ring_points), 0.003f),
            {1, 0.5f, 0.5f, 1.f},
            {1, 0, 0, 1.f},
            {1, 0, 0},
        };
        return &component;
    }
    case interact::rotate_y:
    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 32, ring_points, _countof(ring_points), -0.003f),
            {0.5f, 1, 0.5f, 1.f},
            {0, 1, 0, 1.f},
            {0, 1, 0},
        };
        return &component;
    }
    case interact::rotate_z:
    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 32, ring_points, _countof(ring_points)),
            {0.5f, 0.5f, 1, 1.f},
            {0, 0, 1, 1.f},
            {0, 0, 1},
        };
        return &component;
    }
    }

    return nullptr;
}

bool orientation_gizmo(const gizmo_system &ctx, const std::string &name, fpalg::TRS &trs, bool is_local)
{
    auto &impl = ctx.m_impl;
    auto &t = castalg::ref_cast<rigid_transform>(trs);
    assert(length2(t.orientation) > float(1e-6));
    rigid_transform p = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position); // Orientation is local by default
    auto [gizmo, created] = impl->get_or_create_gizmo(hash_fnv1a(name));
    if (created)
    {
        // TODO
    }

    // update
    interact updated_state = interact::none;
    auto ray = detransform(p, impl->get_ray());
    float best_t = std::numeric_limits<float>::infinity();
    for (auto c : orientation_components)
    {
        auto f = intersect_ray_mesh(ray, get_mesh(c)->mesh);
        if (f < best_t)
        {
            updated_state = c;
            best_t = f;
        }
    }
    if (impl->state.has_clicked)
    {
        auto mesh = get_mesh(updated_state);
        if (mesh)
        {
            auto hit=p.transform_point(ray.origin + ray.direction * best_t);
            gizmo->begin(mesh, hit, t, {});
        }
    }
    if (impl->state.has_released)
    {
        gizmo->end();
    }

    // drag
    gizmo->axisRotationDragger(impl->state, impl->get_ray(), t.position, is_local, &p.orientation);

    // draw
    auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());

    std::array<gizmo_mesh_component *, 1> world_and_active = {
        gizmo->mesh(),
    };
    if (!is_local && gizmo->mesh())
    {
        auto mesh = gizmo->mesh();
        gizmo_renderable r{
            .mesh = mesh->mesh,
            .color = mesh->base_color,
        };
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        impl->drawlist.push_back(r);
    }
    else
    {
        static interact components[] = {
            interact::rotate_x,
            interact::rotate_y,
            interact::rotate_z,
        };

        for (auto c : components)
        {
            auto mesh = get_mesh(c);
            gizmo_renderable r{
                .mesh = mesh->mesh,
                .color = (mesh == gizmo->mesh()) ? mesh->base_color : mesh->highlight_color,
            };
            for (auto &v : r.mesh.vertices)
            {
                v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
                v.normal = transform_vector(modelMatrix, v.normal);
            }
            impl->drawlist.push_back(r);
        }
    }

    // For non-local transformations, we only present one rotation ring
    // and draw an arrow from the center of the gizmo to indicate the degree of rotation
    if (!is_local && gizmo->mesh())
    {
        // Create orthonormal basis for drawing the arrow
        auto a = qrot(p.orientation, gizmo->m_state.originalPositionToClick());
        auto zDir = normalize(gizmo->mesh()->axis), xDir = normalize(cross(a, zDir)), yDir = cross(zDir, xDir);

        // Ad-hoc geometry
        minalg::float2 arrow_points[] = {{0.0f, 0.f}, {0.0f, 0.05f}, {0.8f, 0.05f}, {0.9f, 0.10f}, {1.0f, 0}};
        auto geo = geometry_mesh::make_lathed_geometry(yDir, xDir, zDir, 32, arrow_points, _countof(arrow_points));

        gizmo_renderable r;
        r.mesh = geo;
        r.color = minalg::float4(1);
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position);
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        impl->drawlist.push_back(r);

        t.orientation = qmul(p.orientation, gizmo->m_state.original.orientation);
    }
    else if (is_local == true && gizmo->mesh())
        t.orientation = p.orientation;

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
