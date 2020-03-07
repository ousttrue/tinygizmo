#include "tinygizmo.h"
#include "utilmath.h"
#include "minalg.h"
#include "impl.h"
#include "../screenstate/castalg.h"

namespace tinygizmo
{

static void plane_translation_dragger(Gizmo &gizmo,
                                      const gizmo_application_state &state, const ray &r, const minalg::float3 &plane_normal, minalg::float3 &point)
{
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

static const GizmoComponent *translation_components[] = {
    &componentX,
    &componentY,
    &componentZ,
    &componentXY,
    &componentYZ,
    &componentZX,
    &componentXYZ,
};

std::pair<const GizmoComponent *, float> raycast(const ray &ray)
{
    const GizmoComponent *updated_state = nullptr;
    float best_t = std::numeric_limits<float>::infinity();
    for (auto c : translation_components)
    {
        auto t = intersect_ray_mesh(ray, c->mesh);
        if (t < best_t)
        {
            updated_state = c;
            best_t = t;
        }
    }
    return std::make_pair(updated_state, best_t);
}

void draw(Gizmo &gizmo, gizmo_system_impl *impl, const rigid_transform &p)
{
    auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());

    for (auto c : translation_components)
    {
        gizmo_renderable r{
            .mesh = c->mesh,
            .color = (c == gizmo.mesh()) ? c->base_color : c->highlight_color,
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
    auto [gizmo, created] = impl->get_or_create_gizmo(hash_fnv1a(name));

    // raycast
    auto worldRay = impl->get_ray();
    auto &t = castalg::ref_cast<rigid_transform>(trs);
    auto withoutScale = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position);
    auto localRay = detransform(withoutScale, worldRay);
    auto [mesh, best_t] = raycast(localRay);
    gizmo->hover(mesh != nullptr);

    // update
    if (impl->state.has_clicked)
    {
        if (mesh)
        {
            auto localHit = localRay.origin + localRay.direction * best_t;
            auto worldHit = withoutScale.transform_vector(localHit);
            minalg::float3 axis;
            if (mesh == &componentXYZ)
            {
                axis = -minalg::qzdir(castalg::ref_cast<minalg::float4>(impl->state.camera_orientation));
            }
            else
            {
                if (is_local)
                {
                    axis = withoutScale.transform_vector(mesh->axis);
                }
                else
                {
                    axis = mesh->axis;
                }
            }
            gizmo->begin(mesh, worldHit, t, axis);
        }
    }
    else if (impl->state.has_released)
    {
        gizmo->end();
    }

    // drag
    gizmo->translationDragger(impl->state, worldRay, t.position);

    // draw
    draw(*gizmo, impl, withoutScale);

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
