#include "tinygizmo.h"
#include "impl.h"
#include "assert.h"

namespace tinygizmo
{

using fpalg::operator+;
using fpalg::operator-;
using fpalg::operator*;

static bool dragger(const GizmoComponent &component,
                    const fpalg::Ray &worldRay, const GizmoState &state, float snapVaue,
                    fpalg::Transform *out, bool is_local)
{
    auto start_orientation = is_local ? state.original.rotation : fpalg::float4{0, 0, 0, 1};
    fpalg::Transform original_pose = {state.original.position, start_orientation};
    auto the_axis = original_pose.ApplyDirection(component.axis);
    auto t = worldRay >> fpalg::Plane{the_axis, state.original.position + state.offset};
    if (t < 0)
    {
        return false;
    }

    auto center_of_rotation = state.original.position + the_axis * fpalg::Dot(the_axis, state.offset);
    auto arm1 = fpalg::Normalize(state.original.position + state.offset - center_of_rotation);
    auto arm2 = fpalg::Normalize(worldRay.SetT(t) - center_of_rotation);
    float d = fpalg::Dot(arm1, arm2);
    if (d > 0.999f)
    {
        return false;
    }

    float angle = std::acos(d);
    if (angle < 0.001f)
    {
        return false;
    }

    // if (snapVaue)
    // {
    //     auto snapped = make_rotation_quat_between_vectors_snapped(arm1, arm2, snapVaue);
    //     out->rotation = fpalg::size_cast<fpalg::float4>(qmul(snapped, start_orientation));
    //     return true;
    // }
    // else
    {
        auto a = fpalg::Normalize(fpalg::Cross(arm1, arm2));
        // out->rotation = fpalg::size_cast<fpalg::float4>(qmul(rotation_quat(a, angle), start_orientation));
        out->rotation = fpalg::QuaternionMul(
            fpalg::QuaternionAxisAngle(a, angle),
            start_orientation);
        return true;
    }
}

static fpalg::float2 ring_points[] = {{+0.025f, 1}, {-0.025f, 1}, {-0.025f, 1}, {-0.025f, 1.1f}, {-0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1}};

static GizmoComponent componentX{
    geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 32, ring_points, _countof(ring_points), 0.003f),
    {1, 0.5f, 0.5f, 1.f},
    {1, 0, 0, 1.f},
    {1, 0, 0},
};
static GizmoComponent componentY{
    geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 32, ring_points, _countof(ring_points), -0.003f),
    {0.5f, 1, 0.5f, 1.f},
    {0, 1, 0, 1.f},
    {0, 1, 0},
};
static GizmoComponent componentZ{
    geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 32, ring_points, _countof(ring_points)),
    {0.5f, 0.5f, 1, 1.f},
    {0, 0, 1, 1.f},
    {0, 0, 1},
};

static const GizmoComponent *orientation_components[] = {
    &componentX,
    &componentY,
    &componentZ,
};

inline std::pair<const GizmoComponent *, float> raycast(const fpalg::Ray &ray)
{
    const GizmoComponent *updated_state = nullptr;
    float best_t = std::numeric_limits<float>::infinity();
    for (auto c : orientation_components)
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

static void draw_global_active(std::vector<gizmo_renderable> &drawlist,
                               const fpalg::Transform &gizmoTransform, const GizmoComponent *active,
                               const GizmoState &state)
{
    // auto modelMatrix = fpalg::size_cast<minalg::float4x4>(gizmoTransform.matrix());

    // For non-local transformations, we only present one rotation ring
    // and draw an arrow from the center of the gizmo to indicate the degree of rotation
    gizmo_renderable r{
        .mesh = active->mesh,
        .color = active->base_color,
    };
    for (auto &v : r.mesh.vertices)
    {
        v.position = gizmoTransform.ApplyPosition(v.position); // transform local coordinates into worldspace
        v.normal = gizmoTransform.ApplyDirection(v.normal);
    }
    drawlist.push_back(r);

    {
        // Create orthonormal basis for drawing the arrow
        auto a = gizmoTransform.ApplyDirection(state.offset);
        auto zDir = fpalg::Normalize(active->axis);
        auto xDir = fpalg::Normalize(fpalg::Cross(a, zDir));
        auto yDir = fpalg::Cross(zDir, xDir);

        // Ad-hoc geometry
        fpalg::float2 arrow_points[] = {{0.0f, 0.f}, {0.0f, 0.05f}, {0.8f, 0.05f}, {0.9f, 0.10f}, {1.0f, 0}};
        auto geo = geometry_mesh::make_lathed_geometry(
            fpalg::size_cast<fpalg::float3>(yDir),
            fpalg::size_cast<fpalg::float3>(xDir),
            fpalg::size_cast<fpalg::float3>(zDir),
            32, arrow_points, _countof(arrow_points));

        gizmo_renderable r;
        r.mesh = geo;
        r.color = fpalg::float4{1, 1, 1, 1};
        for (auto &v : r.mesh.vertices)
        {
            v.position = gizmoTransform.ApplyPosition(v.position);
            v.normal = gizmoTransform.ApplyDirection(v.normal);
        }
        drawlist.push_back(r);
    }
}

static void draw(std::vector<gizmo_renderable> &drawlist, const fpalg::Transform &gizmoTransform, const GizmoComponent *active)
{
    for (auto mesh : orientation_components)
    {
        gizmo_renderable r{
            .mesh = mesh->mesh,
            .color = (mesh == active) ? mesh->base_color : mesh->highlight_color,
        };
        for (auto &v : r.mesh.vertices)
        {
            v.position = gizmoTransform.ApplyPosition(v.position);
            v.normal = gizmoTransform.ApplyDirection(v.normal);
        }
        drawlist.push_back(r);
    }
}

bool orientation_gizmo(const gizmo_system &ctx, const std::string &name, fpalg::TRS &trs, bool is_local)
{
    auto &impl = ctx.m_impl;
    auto [gizmo, created] = impl->get_or_create_gizmo(hash_fnv1a(name));

    // assert(length2(t.orientation) > float(1e-6));
    auto worldRay = impl->get_ray();
    fpalg::Transform gizmoTransform = is_local ? trs.transform : fpalg::Transform{trs.position, {0, 0, 0, 1}}; // Orientation is local by default

    // raycast
    {
        auto localRay = worldRay.Transform(gizmoTransform.Inverse());
        auto [mesh, best_t] = raycast(localRay);

        // update
        if (impl->state.has_clicked)
        {
            if (mesh)
            {
                auto localHit = localRay.SetT(best_t);
                using fpalg::operator-;
                auto offset = gizmoTransform.ApplyPosition(localHit) - trs.position;
                gizmo->begin(mesh, offset, trs, {});
            }
        }
        else if (impl->state.has_released)
        {
            gizmo->end();
        }
    }

    // drag
    auto active = gizmo->active();
    if (active)
    {
        dragger(*active, worldRay, gizmo->m_state, impl->state.snap_rotation, &gizmoTransform, is_local);
        if (!is_local)
        {
            trs.rotation = fpalg::QuaternionMul(gizmoTransform.rotation, gizmo->m_state.original.rotation);
        }
        else
        {
            trs.rotation = gizmoTransform.rotation;
        }
    }

    // draw
    if (!is_local && active)
    {
        draw_global_active(impl->drawlist, gizmoTransform, active, gizmo->m_state);
    }
    else
    {
        draw(impl->drawlist, gizmoTransform, active);
    }

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
