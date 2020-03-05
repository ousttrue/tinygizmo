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
    static std::vector<minalg::float2> ring_points = {{+0.025f, 1}, {-0.025f, 1}, {-0.025f, 1}, {-0.025f, 1.1f}, {-0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1}};

    switch (c)
    {
    case interact::rotate_x:
    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 32, ring_points, 0.003f),
            {1, 0.5f, 0.5f, 1.f},
            {1, 0, 0, 1.f},
            {1, 0, 0},
        };
        return &component;
    }
    case interact::rotate_y:
    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 32, ring_points, -0.003f),
            {0.5f, 1, 0.5f, 1.f},
            {0, 1, 0, 1.f},
            {0, 1, 0},
        };
        return &component;
    }
    case interact::rotate_z:
    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 32, ring_points),
            {0.5f, 0.5f, 1, 1.f},
            {0, 0, 1, 1.f},
            {0, 0, 1},
        };
        return &component;
    }
    }

    return nullptr;
}

minalg::float4 axis_rotation_dragger(interaction_state &gizmo,
                                     const gizmo_application_state &state, const ray &r,
                                     const minalg::float3 &axis, const minalg::float3 &center, const minalg::float4 &start_orientation)
{
    if (state.mouse_left)
    {
        rigid_transform original_pose = {start_orientation, gizmo.original_position};
        auto the_axis = original_pose.transform_vector(axis);
        minalg::float4 the_plane = {the_axis, -dot(the_axis, gizmo.click_offset)};

        float t;
        if (!intersect_ray_plane(r, the_plane, &t))
        {
            return start_orientation;
        }

        auto center_of_rotation = gizmo.original_position + the_axis * dot(the_axis, gizmo.click_offset - gizmo.original_position);
        auto arm1 = normalize(gizmo.click_offset - center_of_rotation);
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

static minalg::float4 dragger(interaction_state &gizmo, const gizmo_application_state &state, const ray &ray,
                              const minalg::float3 &center, bool is_local)
{
    auto starting_orientation = is_local ? gizmo.original_orientation : minalg::float4(0, 0, 0, 1);
    return axis_rotation_dragger(gizmo, state, ray, gizmo.mesh->axis, center, starting_orientation);
}

bool orientation_gizmo(const gizmo_system &ctx, const std::string &name, fpalg::TRS &trs, bool is_local)
{
    auto &impl = ctx.m_impl;
    auto &t = castalg::ref_cast<rigid_transform>(trs);
    assert(length2(t.orientation) > float(1e-6));
    rigid_transform p = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position); // Orientation is local by default
    const uint32_t id = hash_fnv1a(name);
    auto &gizmo = impl->gizmos[id];

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
        gizmo.mesh = get_mesh(updated_state);
        if (gizmo.mesh)
        {
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
        gizmo.mesh = nullptr;
        gizmo.active = false;
    }

    // drag
    if (gizmo.active)
    {
        p.orientation = dragger(gizmo, impl->state, impl->get_ray(), t.position, is_local);
    }

    // draw
    auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());

    std::array<gizmo_mesh_component *, 1> world_and_active = {
        gizmo.mesh,
    };
    if (!is_local && gizmo.mesh)
    {
        auto mesh = gizmo.mesh;
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
                .color = (mesh == gizmo.mesh) ? mesh->base_color : mesh->highlight_color,
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
    if (!is_local && gizmo.mesh)
    {
        interaction_state &interaction = gizmo;

        // Create orthonormal basis for drawing the arrow
        auto a = qrot(p.orientation, interaction.click_offset - interaction.original_position);
        auto zDir = normalize(gizmo.mesh->axis), xDir = normalize(cross(a, zDir)), yDir = cross(zDir, xDir);

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
    else if (is_local == true && gizmo.mesh)
        t.orientation = p.orientation;

    return (gizmo.hover || gizmo.active);
}

} // namespace tinygizmo
