#include "tinygizmo.h"
#include "utilmath.h"
#include "impl.h"
#include <iostream>

namespace tinygizmo
{
static minalg::float2 mace_points[] = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.1f}, {1.25f, 0.1f}, {1.25f, 0}};

static GizmoComponent xComponent{
    geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, mace_points, _countof(mace_points)),
    {1, 0.5f, 0.5f, 1.f},
    {1, 0, 0, 1.f},
    {1, 0, 0},
};
static GizmoComponent yComponent{
    geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, mace_points, _countof(mace_points)),
    {0.5f, 1, 0.5f, 1.f},
    {0, 1, 0, 1.f},
    {0, 1, 0},
};
static GizmoComponent zComponent{
    geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, mace_points, _countof(mace_points)),
    {0.5f, 0.5f, 1, 1.f},
    {0, 0, 1, 1.f},
    {0, 0, 1},
};

static GizmoComponent *g_meshes[] = {&xComponent, &yComponent, &zComponent};

class ScaleGizmo : public Gizmo
{
public:
    void onClick(const ray &ray, const rigid_transform &t) override
    {
        GizmoComponent *updated_state = nullptr;
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

        if (updated_state)
        {
            rigid_transform withoutScale(t.orientation, t.position);
            begin(updated_state, withoutScale.transform_point(ray.origin + ray.direction * best_t), t, {});
        }
    }

    void draw(const fpalg::Transform &t, std::vector<gizmo_renderable> &drawlist) override
    {
        // rigid_transform withoutScale(t.orientation, t.position);
        // auto modelMatrix = fpalg::size_cast<minalg::float4x4>(withoutScale.matrix());
        // auto modelMatrix = t.Matrix();
        for (auto mesh : g_meshes)
        {
            gizmo_renderable r{
                .mesh = mesh->mesh,
                .color = (mesh == m_mesh) ? mesh->base_color : mesh->highlight_color,
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
};

bool scale_gizmo(const gizmo_system &ctx, const std::string &name, fpalg::TRS &trs, bool is_uniform)
{
    auto &impl = ctx.m_impl;

    auto id = hash_fnv1a(name);
    auto gizmo = impl->get_gizmo(id);
    if (!gizmo)
    {
        gizmo = new ScaleGizmo();
        impl->add_gizmo(id, gizmo);
    }

    auto worldRay = fpalg::size_cast<fpalg::Ray>(impl->get_ray());
    auto localRay = worldRay.ToLocal(trs.transform);

    if (impl->state.has_clicked)
    {
        gizmo->onClick(fpalg::size_cast<ray>(localRay), fpalg::size_cast<rigid_transform>(trs));
    }
    if (impl->state.has_released)
    {
        gizmo->end();
    }

    auto active = gizmo->mesh();
    if (active)
    {
        if (active->axisScaleDragger(fpalg::size_cast<ray>(localRay), gizmo->m_state, is_uniform, &fpalg::size_cast<minalg::float3>(trs.scale)))

        {
            // if (impl->state.snap_scale)
            // {
            //     trs.scale = snap(trs.scale, impl->state.snap_scale);
            // }
        }
    }

    gizmo->draw(trs.transform, impl->drawlist);

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
