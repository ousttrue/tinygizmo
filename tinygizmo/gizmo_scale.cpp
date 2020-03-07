#include "tinygizmo.h"
#include "utilmath.h"
#include "impl.h"
#include <iostream>

namespace tinygizmo
{
class ScaleGizmoComponent : public GizmoComponent
{
public:
    using GizmoComponent::GizmoComponent;

    bool dragger(
        const ray &ray, const GizmoState &state,
        const bool uniform,
        rigid_transform *out) const override
    {
        // auto axis = m_mesh->axis;
        auto plane_tangent = cross(axis, state.original.position - fpalg::size_cast<minalg::float3>(ray.origin));
        auto plane_normal = cross(axis, plane_tangent);

        // If an intersection exists between the ray and the plane, place the object at that point
        const float denom = dot(ray.direction, plane_normal);
        if (std::abs(denom) == 0)
        {
            return false;
        }

        const float t = dot(state.original.position - ray.origin, plane_normal) / denom;
        if (t < 0)
        {
            return false;
        }

        auto distance = ray.origin + ray.direction * t;

        auto hoge = (distance - state.click);
        auto offset_on_axis = hoge * axis;
        flush_to_zero(offset_on_axis);
        // std::cout << offset_on_axis << std::endl;
        auto new_scale = state.original.scale + offset_on_axis;

        if (uniform)
            out->scale = minalg::float3(clamp(dot(distance, new_scale), 0.01f, 1000.f));
        else
            out->scale = minalg::float3(clamp(new_scale.x, 0.01f, 1000.f), clamp(new_scale.y, 0.01f, 1000.f), clamp(new_scale.z, 0.01f, 1000.f));
        return true;
    }
};

static minalg::float2 mace_points[] = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.1f}, {1.25f, 0.1f}, {1.25f, 0}};

static ScaleGizmoComponent xComponent(
    geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, mace_points, _countof(mace_points)),
    {1, 0.5f, 0.5f, 1.f},
    {1, 0, 0, 1.f},
    {1, 0, 0});
static ScaleGizmoComponent yComponent{
    geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, mace_points, _countof(mace_points)),
    {0.5f, 1, 0.5f, 1.f},
    {0, 1, 0, 1.f},
    {0, 1, 0},
};
static ScaleGizmoComponent zComponent{
    geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, mace_points, _countof(mace_points)),
    {0.5f, 0.5f, 1, 1.f},
    {0, 0, 1, 1.f},
    {0, 0, 1},
};

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
            auto hit = localRay.origin + (localRay.direction * best_t);
            rigid_transform withoutScale(t.orientation, t.position);
            auto worldHit = withoutScale.transform_point(hit);
            gizmo->begin(updated_state, worldHit, t, {});
        }
    }
    else if (impl->state.has_released)
    {
        gizmo->end();
    }

    auto active = gizmo->activeMesh();
    if (active)
    {
        if (active->dragger(fpalg::size_cast<ray>(localRay), gizmo->m_state, is_uniform, &fpalg::size_cast<rigid_transform>(trs)))

        {
            // if (impl->state.snap_scale)
            // {
            //     trs.scale = snap(trs.scale, impl->state.snap_scale);
            // }
        }
    }

    draw(trs.transform, impl->drawlist, gizmo->activeMesh());

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
