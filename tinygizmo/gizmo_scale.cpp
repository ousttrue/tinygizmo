#include "tinygizmo.h"
#include "utilmath.h"
#include "impl.h"
#include <iostream>

namespace tinygizmo
{

static bool dragger(const GizmoComponent &component,
                    const ray &worldRay, const GizmoState &state, float snapValue,
                    rigid_transform *out, bool uniform)
{
    // auto axis = m_mesh->axis;
    auto plane_tangent = cross(component.axis, state.original.position - fpalg::size_cast<minalg::float3>(worldRay.origin));
    auto N = cross(component.axis, plane_tangent);

    // If an intersection exists between the ray and the plane, place the object at that point
    auto NR = dot(N, worldRay.direction);
    if (std::abs(NR) == 0)
    {
        return false;
    }

    auto Q = state.original.position + state.offset - worldRay.origin;
    const float t = dot(N, Q) / NR;
    if (t < 0)
    {
        return false;
    }

    auto intersect = worldRay.origin + worldRay.direction * t;

    auto offset_on_axis = (intersect - state.offset - state.original.position) * component.axis;
    flush_to_zero(offset_on_axis);
    auto new_scale = state.original.scale + offset_on_axis;

    if (uniform)
        out->scale = minalg::float3(clamp(dot(intersect, new_scale), 0.01f, 1000.f));
    else
        out->scale = minalg::float3(
            clamp(new_scale.x, 0.01f, 1000.f),
            clamp(new_scale.y, 0.01f, 1000.f),
            clamp(new_scale.z, 0.01f, 1000.f));

    if (snapValue)
    {
        out->scale = snap(out->scale, snapValue);
    }

    return true;
}

static minalg::float2 mace_points[] = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.1f}, {1.25f, 0.1f}, {1.25f, 0}};

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

std::pair<const GizmoComponent *, float> raycast(const ray &ray, const rigid_transform &t)
{
    const GizmoComponent *updated_state = nullptr;
    float best_t = std::numeric_limits<float>::infinity();
    for (auto mesh : g_meshes)
    {
        auto t = intersect_ray_mesh(ray, mesh->mesh);
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
            v.position = fpalg::size_cast<minalg::float3>(t.ApplyPosition(fpalg::size_cast<std::array<float, 3>>(v.position)));
            v.normal = fpalg::size_cast<minalg::float3>(t.ApplyDirection(fpalg::size_cast<std::array<float, 3>>(v.normal)));
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
    auto worldRay = impl->get_ray();
    auto &t = fpalg::size_cast<rigid_transform>(trs);
    auto withoutScale = rigid_transform(t.orientation, t.position);
    auto localRay = detransform(withoutScale, worldRay);

    if (impl->state.has_clicked)
    {
        auto &t = fpalg::size_cast<rigid_transform>(trs);
        auto [updated_state, best_t] = raycast(fpalg::size_cast<ray>(localRay), t);

        if (updated_state)
        {
            auto localHit = localRay.origin + (localRay.direction * best_t);
            rigid_transform withoutScale(t.orientation, t.position);
            auto offset = withoutScale.transform_point(localHit) - t.position;
            gizmo->begin(updated_state, offset, t, {});
        }
    }
    else if (impl->state.has_released)
    {
        gizmo->end();
    }

    auto active = gizmo->active();
    if (active)
    {
        if (dragger(*active, fpalg::size_cast<ray>(localRay), gizmo->m_state, impl->state.snap_scale,
                    &fpalg::size_cast<rigid_transform>(trs), is_uniform))
        {
        }
    }

    draw(trs.transform, impl->drawlist, active);

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
