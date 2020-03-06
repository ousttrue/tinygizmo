#include "tinygizmo.h"
#include "utilmath.h"
#include "impl.h"
#include <iostream>

namespace tinygizmo
{

static void addMeshes(gizmo *gizmo)
{
    static std::vector<minalg::float2> mace_points = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.1f}, {1.25f, 0.1f}, {1.25f, 0}};

    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, mace_points),
            {1, 0.5f, 0.5f, 1.f},
            {1, 0, 0, 1.f},
            {1, 0, 0},
        };
        gizmo->addMesh(&component);
    }

    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, mace_points),
            {0.5f, 1, 0.5f, 1.f},
            {0, 1, 0, 1.f},
            {0, 1, 0},
        };
        gizmo->addMesh(&component);
    }

    {
        static gizmo_mesh_component component{
            geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, mace_points),
            {0.5f, 0.5f, 1, 1.f},
            {0, 0, 1, 1.f},
            {0, 0, 1},
        };
        gizmo->addMesh(&component);
    }
}

bool scale_gizmo(const gizmo_system &ctx, const std::string &name, fpalg::TRS &trs, bool is_uniform)
{
    auto &impl = ctx.m_impl;

    auto [gizmo, created] = impl->get_or_create_gizmo(hash_fnv1a(name));
    if (created)
    {
        addMeshes(gizmo);
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

    gizmo->axisScaleDragger(impl->state, fpalg::size_cast<ray>(localRay), fpalg::size_cast<minalg::float3>(trs.transform.position), is_uniform, &fpalg::size_cast<minalg::float3>(trs.scale));

    gizmo->draw(trs.transform, impl->drawlist);

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
