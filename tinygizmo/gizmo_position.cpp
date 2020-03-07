#include "tinygizmo.h"
#include "utilmath.h"
#include "minalg.h"
#include "impl.h"
#include "../screenstate/castalg.h"

namespace tinygizmo
{

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
};

static const interact translation_components[] = {
    interact::translate_x,
    interact::translate_y,
    interact::translate_z,
    interact::translate_xy,
    interact::translate_yz,
    interact::translate_zx,
    interact::translate_xyz,
};

static void plane_translation_dragger(Gizmo &gizmo,
                                      const gizmo_application_state &state, const ray &r, const minalg::float3 &plane_normal, minalg::float3 &point)
{
    // Mouse clicked
    if (state.has_clicked)
        gizmo.m_state.original.position = point;

    if (state.mouse_left)
    {
        // Define the plane to contain the original position of the object
        auto plane_point = gizmo.m_state.original.position;

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

static void axis_translation_dragger(Gizmo &gizmo,
                                     const gizmo_application_state &state, const ray &ray, const minalg::float3 &axis, minalg::float3 &point)
{
    if (state.mouse_left)
    {
        // First apply a plane translation dragger with a plane that contains the desired axis and is oriented to face the camera
        auto plane_tangent = minalg::cross(axis, point - castalg::ref_cast<minalg::float3>(state.camera_position));
        auto plane_normal = cross(axis, plane_tangent);
        plane_translation_dragger(gizmo, state, ray, plane_normal, point);

        // Constrain object motion to be along the desired axis
        point = gizmo.m_state.original.position + axis * dot(point - gizmo.m_state.original.position, axis);
    }
}

static minalg::float2 arrow_points[] = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.10f}, {1.2f, 0}};
static GizmoComponent componentX(
    geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, arrow_points, _countof(arrow_points)),
    {1, 0.5f, 0.5f, 1.f},
    {1, 0, 0, 1.f},
    {1, 0, 0},
    axis_translation_dragger);
static GizmoComponent componentY(
    geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, arrow_points, _countof(arrow_points)),
    {0.5f, 1, 0.5f, 1.f},
    {0, 1, 0, 1.f},
    {0, 1, 0},
    axis_translation_dragger);
static GizmoComponent componentZ(
    geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, arrow_points, _countof(arrow_points)),
    {0.5f, 0.5f, 1, 1.f},
    {0, 0, 1, 1.f},
    {0, 0, 1},
    axis_translation_dragger);
static GizmoComponent componentXY(
    geometry_mesh::make_box_geometry({0.25, 0.25, -0.01f}, {0.75f, 0.75f, 0.01f}),
    {1, 1, 0.5f, 0.5f},
    {1, 1, 0, 0.6f},
    {0, 0, 1},
    plane_translation_dragger);
static GizmoComponent componentYZ(
    geometry_mesh::make_box_geometry({-0.01f, 0.25, 0.25}, {0.01f, 0.75f, 0.75f}),
    {0.5f, 1, 1, 0.5f},
    {0, 1, 1, 0.6f},
    {1, 0, 0},
    plane_translation_dragger);
static GizmoComponent componentZX(
    geometry_mesh::make_box_geometry({0.25, -0.01f, 0.25}, {0.75f, 0.01f, 0.75f}),
    {1, 0.5f, 1, 0.5f},
    {1, 0, 1, 0.6f},
    {0, 1, 0},
    plane_translation_dragger);
static GizmoComponent componentXYZ(
    geometry_mesh::make_box_geometry({-0.05f, -0.05f, -0.05f}, {0.05f, 0.05f, 0.05f}),
    {0.9f, 0.9f, 0.9f, 0.25f},
    {1, 1, 1, 0.35f},
    {0, 0, 0},
    plane_translation_dragger);

static GizmoComponent *
get_mesh(interact c)
{
    switch (c)
    {
    case interact::translate_x:
    {
        return &componentX;
    }
    case interact::translate_y:
    {
        return &componentY;
    }
    case interact::translate_z:
    {
        return &componentZ;
    }
    case interact::translate_xy:
    {
        return &componentXY;
    }
    case interact::translate_yz:
    {
        return &componentYZ;
    }
    case interact::translate_zx:
    {
        return &componentZX;
    }
    case interact::translate_xyz:
    {
        return &componentXYZ;
    }
    }

    return nullptr;
}

// check hit
std::pair<GizmoComponent *, float> raycast(const ray &worldRay, Gizmo &gizmo, const gizmo_application_state &state, const rigid_transform &t, bool is_local)
{
    auto p = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position);
    auto ray = detransform(p, worldRay);

    interact updated_state = interact::none;
    float best_t = std::numeric_limits<float>::infinity();
    bool hover = false;
    for (auto c : translation_components)
    {
        auto t = intersect_ray_mesh(ray, get_mesh(c)->mesh);
        if (t < best_t)
        {
            updated_state = c;
            best_t = t;
            hover = true;
        }
    }
    gizmo.hover(hover);

    return std::make_pair(get_mesh(updated_state), best_t);
}

void draw(Gizmo &gizmo, gizmo_system_impl *impl, const rigid_transform &p)
{
    auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());

    for (auto c : translation_components)
    {
        auto mesh = get_mesh(c);
        gizmo_renderable r{
            .mesh = mesh->mesh,
            .color = (mesh == gizmo.mesh()) ? mesh->base_color : mesh->highlight_color,
        };
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        impl->drawlist.push_back(r);
    }
}

bool position_gizmo(const gizmo_system &ctx, const std::string &name, fpalg::TRS &trs, bool is_local)
{
    auto &impl = ctx.m_impl;
    auto &t = castalg::ref_cast<rigid_transform>(trs);
    auto [gizmo, created] = impl->get_or_create_gizmo(hash_fnv1a(name));

    // update
    auto worldRay = impl->get_ray();
    auto [mesh, best_t] = raycast(worldRay, *gizmo, impl->state, t, is_local);

    if (impl->state.has_clicked)
    {
        if (mesh)
        {
            auto p = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position);
            auto ray = detransform(p, worldRay);

            auto offset = is_local ? p.transform_vector(ray.origin + ray.direction * best_t) : ray.origin + ray.direction * best_t;
            minalg::float3 axis;
            if (mesh == &componentXYZ)
            {
                axis = -minalg::qzdir(castalg::ref_cast<minalg::float4>(impl->state.camera_orientation));
            }
            else
            {
                if (is_local)
                {
                    // minalg::float3 local_axes[3]{qxdir(p.orientation), qydir(p.orientation), qzdir(p.orientation)};
                    axis = p.transform_vector(mesh->axis);
                }
                else
                {
                    axis = mesh->axis;
                }
            }
            gizmo->begin(mesh, offset, t, axis);
        }
    }

    if (impl->state.has_released)
    {
        gizmo->end();
    }

    // drag
    gizmo->translationDragger(impl->state, impl->get_ray(), t.position);

    // draw
    auto p = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position);
    draw(*gizmo, impl, p);

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
