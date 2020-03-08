#include "tinygizmo.h"
#include "utilmath.h"
#include "impl.h"
#include <iostream>

namespace tinygizmo
{

using fpalg::operator-;
using fpalg::operator+;
using fpalg::operator*;

static bool dragger(const GizmoComponent &component,
                    const fpalg::Ray &worldRay, const GizmoState &state, float snapValue,
                    rigid_transform *out, bool uniform)
{
    auto plane_tangent = fpalg::Cross(
        fpalg::size_cast<fpalg::float3>(component.axis),
        fpalg::size_cast<fpalg::float3>(state.original.position) - worldRay.origin);
    auto N = fpalg::Cross(fpalg::size_cast<fpalg::float3>(component.axis), plane_tangent);

    // If an intersection exists between the ray and the plane, place the object at that point
    auto t = worldRay >> fpalg::Plane{
                             N,
                             fpalg::size_cast<fpalg::float3>(state.original.position + state.offset)};
    if (t < 0)
    {
        return false;
    }
    auto intersect = worldRay.SetT(t);

    auto offset_on_axis = (intersect - fpalg::size_cast<fpalg::float3>(state.offset - state.original.position)) * fpalg::size_cast<fpalg::float3>(component.axis);
    flush_to_zero(fpalg::size_cast<minalg::float3>(offset_on_axis));
    auto new_scale = fpalg::size_cast<fpalg::float3>(state.original.scale) + offset_on_axis;

    if (uniform)
        out->scale = minalg::float3(clamp(fpalg::Dot(intersect, new_scale), 0.01f, 1000.f));
    else
        out->scale = minalg::float3(
            clamp(new_scale[0], 0.01f, 1000.f),
            clamp(new_scale[1], 0.01f, 1000.f),
            clamp(new_scale[2], 0.01f, 1000.f));

    // if (snapValue)
    // {
    //     out->scale = snap(out->scale, snapValue);
    // }

    return true;
}

static fpalg::float2 mace_points[] = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.1f}, {1.25f, 0.1f}, {1.25f, 0}};

static GizmoComponent xComponent{
    geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, mace_points, _countof(mace_points)),
    {1, 0.5f, 0.5f, 1.f},
    {1, 0, 0, 1.f},
    {1, 0, 0}};
static GizmoComponent yComponent{
    geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, mace_points, _countof(mace_points)),
    {0.5f, 1, 0.5f, 1.f},
    {0, 1, 0, 1.f},
    {0, 1, 0}};
static GizmoComponent zComponent{
    geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, mace_points, _countof(mace_points)),
    {0.5f, 0.5f, 1, 1.f},
    {0, 0, 1, 1.f},
    {0, 0, 1}};

static const GizmoComponent *g_meshes[] = {&xComponent, &yComponent, &zComponent};

std::pair<const GizmoComponent *, float> raycast(const fpalg::Ray &ray, const rigid_transform &t)
{
    const GizmoComponent *updated_state = nullptr;
    float best_t = std::numeric_limits<float>::infinity();
    for (auto mesh : g_meshes)
    {
        auto t = ray >> mesh->mesh;
        if (t < best_t)
        {
            updated_state = mesh;
            best_t = t;
        }
    }
    return std::make_pair(updated_state, best_t);
}

void draw(const fpalg::Transform &t, std::vector<gizmo_renderable> &drawlist, const GizmoComponent *activeMesh)
{
    for (auto mesh : g_meshes)
    {
        gizmo_renderable r{
            .mesh = mesh->mesh,
            .color = (mesh == activeMesh) ? mesh->base_color : mesh->highlight_color,
        };
        for (auto &v : r.mesh.vertices)
        {
            // transform local coordinates into worldspace
            v.position = t.ApplyPosition(v.position);
            v.normal = t.ApplyDirection(v.normal);
        }
        drawlist.push_back(r);
    }
}

bool scale_gizmo(const gizmo_system &ctx, const std::string &name, fpalg::TRS &trs, bool is_uniform)
{
    auto &impl = ctx.m_impl;

    auto id = hash_fnv1a(name);
    auto [gizmo, created] = impl->get_or_create_gizmo(id);
    if (!created)
    {
    }

    // auto localRay = impl->get_local_ray(trs.transform);
    auto worldRay = fpalg::size_cast<fpalg::Ray>(impl->get_ray());
    // auto &t = fpalg::size_cast<rigid_transform>(trs);
    // auto withoutScale = rigid_transform(t.orientation, t.position);
    auto localRay = worldRay.Transform(trs.transform.Inverse());

    if (impl->state.has_clicked)
    {
        auto &t = fpalg::size_cast<rigid_transform>(trs);
        auto [updated_state, best_t] = raycast(fpalg::size_cast<fpalg::Ray>(localRay), t);

        if (updated_state)
        {
            auto localHit = localRay.SetT(best_t);
            // rigid_transform withoutScale(t.orientation, t.position);
            using fpalg::operator-;
            auto offset = trs.transform.ApplyPosition(localHit) - trs.transform.position;
            gizmo->begin(updated_state, offset, trs, {});
        }
    }
    else if (impl->state.has_released)
    {
        gizmo->end();
    }

    auto active = gizmo->active();
    if (active)
    {
        if (dragger(*active, localRay, gizmo->m_state, impl->state.snap_scale,
                    &fpalg::size_cast<rigid_transform>(trs), is_uniform))
        {
        }
    }

    draw(trs.transform, impl->drawlist, active);

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
