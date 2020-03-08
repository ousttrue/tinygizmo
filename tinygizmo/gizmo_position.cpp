#include "tinygizmo.h"
#include "minalg.h"
#include "impl.h"


namespace tinygizmo
{

struct Param
{
    minalg::float3 N;
    float snapValue;
};

using fpalg::operator+;
using fpalg::operator-;
using fpalg::operator*;

static bool planeDragger(const GizmoComponent &component, const fpalg::Ray &worldRay, const GizmoState &state, float snapValue,
                         fpalg::Transform *out, const fpalg::float3 &N)
{

    fpalg::Plane plane(N, state.original.position + state.offset);
    auto t = worldRay >> plane;
    if (t < 0)
    {
        return false;
    }

    using namespace fpalg;
    auto intersect = worldRay.SetT(t);
    out->position = intersect - state.offset;

    // if (snapValue)
    // {
    //     out->position = snap(out->position, snapValue);
    // }

    return true;
}

static bool axisDragger(const GizmoComponent &component, const fpalg::Ray &worldRay, const GizmoState &state, float snapValue,
                        fpalg::Transform *out, const fpalg::float3 &axis)
{
    // First apply a plane translation dragger with a plane that contains the desired axis and is oriented to face the camera
    auto plane_tangent = fpalg::Cross(axis, out->position - worldRay.origin);
    auto plane_normal = fpalg::Cross(axis, plane_tangent);
    if (!planeDragger(component, worldRay, state, snapValue, out, plane_normal))
    {
        return false;
    }

    // Constrain object motion to be along the desired axis
    out->position = state.original.position + axis * fpalg::Dot(out->position - state.original.position, axis);
    return true;
}

static fpalg::float2 arrow_points[] = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.10f}, {1.2f, 0}};
static GizmoComponent componentX{
    geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, arrow_points, _countof(arrow_points)),
    {1, 0.5f, 0.5f, 1.f},
    {1, 0, 0, 1.f},
    {1, 0, 0}};
static GizmoComponent componentY{
    geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, arrow_points, _countof(arrow_points)),
    {0.5f, 1, 0.5f, 1.f},
    {0, 1, 0, 1.f},
    {0, 1, 0}};
static GizmoComponent componentZ{
    geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, arrow_points, _countof(arrow_points)),
    {0.5f, 0.5f, 1, 1.f},
    {0, 0, 1, 1.f},
    {0, 0, 1}};
static GizmoComponent componentXY{
    geometry_mesh::make_box_geometry({0.25, 0.25, -0.01f}, {0.75f, 0.75f, 0.01f}),
    {1, 1, 0.5f, 0.5f},
    {1, 1, 0, 0.6f},
    {0, 0, 1}};
static GizmoComponent componentYZ{
    geometry_mesh::make_box_geometry({-0.01f, 0.25, 0.25}, {0.01f, 0.75f, 0.75f}),
    {0.5f, 1, 1, 0.5f},
    {0, 1, 1, 0.6f},
    {1, 0, 0}};
static GizmoComponent componentZX{
    geometry_mesh::make_box_geometry({0.25, -0.01f, 0.25}, {0.75f, 0.01f, 0.75f}),
    {1, 0.5f, 1, 0.5f},
    {1, 0, 1, 0.6f},
    {0, 1, 0}};
static GizmoComponent componentXYZ{
    geometry_mesh::make_box_geometry({-0.05f, -0.05f, -0.05f}, {0.05f, 0.05f, 0.05f}),
    {0.9f, 0.9f, 0.9f, 0.25f},
    {1, 1, 1, 0.35f},
    {0, 0, 0}};

static const GizmoComponent *translation_components[] = {
    &componentX,
    &componentY,
    &componentZ,
    &componentXY,
    &componentYZ,
    &componentZX,
    &componentXYZ,
};

static std::pair<const GizmoComponent *, float> raycast(const fpalg::Ray &ray)
{
    const GizmoComponent *updated_state = nullptr;
    float best_t = std::numeric_limits<float>::infinity();
    for (auto c : translation_components)
    {
        auto t = ray >> c->mesh;
        if (t < best_t)
        {
            updated_state = c;
            best_t = t;
        }
    }
    return std::make_pair(updated_state, best_t);
}

static void draw(Gizmo &gizmo, gizmo_system_impl *impl, const fpalg::Transform &t)
{
    for (auto c : translation_components)
    {
        gizmo_renderable r{
            .mesh = c->mesh,
            .color = (c == gizmo.active()) ? c->base_color : c->highlight_color,
        };
        for (auto &v : r.mesh.vertices)
        {
            v.position = t.ApplyPosition(v.position); // transform local coordinates into worldspace
            v.normal = t.ApplyDirection(v.normal);
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
    auto gizmoTransform = is_local ? trs.transform : fpalg::Transform{trs.transform.position, {0, 0, 0, 1}};
    auto localRay = worldRay.Transform(gizmoTransform.Inverse());
    auto [mesh, best_t] = raycast(localRay);
    gizmo->hover(mesh != nullptr);

    // update
    if (impl->state.has_clicked)
    {
        if (mesh)
        {
            auto localHit = localRay.SetT(best_t);
            using fpalg::operator-;
            auto worldOffset = gizmoTransform.ApplyPosition(localHit) - trs.transform.position;
            fpalg::float3 axis;
            if (mesh == &componentXYZ)
            {
                axis = -fpalg::QuaternionZDir(impl->state.camera_orientation);
            }
            else
            {
                if (is_local)
                {
                    axis = gizmoTransform.ApplyDirection(mesh->axis);
                }
                else
                {
                    axis = mesh->axis;
                }
            }
            gizmo->begin(mesh, worldOffset, trs, axis);
        }
    }
    else if (impl->state.has_released)
    {
        gizmo->end();
    }

    // drag
    auto active = gizmo->active();
    if (active)
    {
        if (active == &componentX || active == &componentY || active == &componentZ)
        {
            axisDragger(*active, worldRay, gizmo->m_state, impl->state.snap_translation, &trs.transform, gizmo->m_state.axis);
        }
        else
        {
            planeDragger(*active, worldRay, gizmo->m_state, impl->state.snap_translation, &trs.transform, gizmo->m_state.axis);
        }
    }

    // draw
    draw(*gizmo, impl, gizmoTransform);

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
