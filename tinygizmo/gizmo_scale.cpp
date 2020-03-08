#include "tinygizmo.h"
#include "impl.h"
#include <iostream>

namespace tinygizmo
{

using fpalg::operator-;
using fpalg::operator+;
using fpalg::operator*;

static void flush_to_zero(minalg::float3 &f)
{
    if (std::abs(f.x) < 0.02f)
        f.x = 0.f;
    if (std::abs(f.y) < 0.02f)
        f.y = 0.f;
    if (std::abs(f.z) < 0.02f)
        f.z = 0.f;
}

template <typename T>
T clamp(const T &val, const T &min, const T &max) { return std::min(std::max(val, min), max); }

static bool dragger(const GizmoComponent &component,
                    const fpalg::Ray &worldRay, const GizmoState &state, float snapValue,
                    fpalg::TRS *out, bool uniform)
{
    auto plane_tangent = fpalg::Cross(
        component.axis,
        state.original.position - worldRay.origin);
    auto N = fpalg::Cross(component.axis, plane_tangent);

    // If an intersection exists between the ray and the plane, place the object at that point
    auto t = worldRay >> fpalg::Plane{
                             N,
                             state.original.position + state.offset};
    if (t < 0)
    {
        return false;
    }
    auto intersect = worldRay.SetT(t);

    auto offset_on_axis = fpalg::size_cast<minalg::float3>((intersect - state.offset - state.original.position) * component.axis);
    flush_to_zero(offset_on_axis);
    auto new_scale = state.original.scale + fpalg::size_cast<fpalg::float3>(offset_on_axis);

    if (uniform)
    {
        auto s = clamp(fpalg::Dot(intersect, new_scale), 0.01f, 1000.f);
        out->scale = fpalg::float3{s, s, s};
    }
    else
    {
        out->scale = fpalg::float3{
            clamp(new_scale[0], 0.01f, 1000.f),
            clamp(new_scale[1], 0.01f, 1000.f),
            clamp(new_scale[2], 0.01f, 1000.f)};
    }

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

static std::pair<const GizmoComponent *, float> raycast(const fpalg::Ray &ray)
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

static void draw(const fpalg::Transform &t, std::vector<gizmo_renderable> &drawlist, const GizmoComponent *activeMesh)
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

bool scale_gizmo(const gizmo_system &ctx, uint32_t id, fpalg::TRS &trs, bool is_uniform)
{
    auto &impl = ctx.m_impl;
    auto [gizmo, created] = impl->get_or_create_gizmo(id);

    auto worldRay = impl->get_ray();
    auto localRay = worldRay.Transform(trs.transform.Inverse());

    if (impl->state.has_clicked)
    {
        auto [updated_state, best_t] = raycast(localRay);

        if (updated_state)
        {
            auto localHit = localRay.SetT(best_t);
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
                    &trs, is_uniform))
        {
        }
    }

    draw(trs.transform, impl->drawlist, active);

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
