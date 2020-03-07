#include "tinygizmo.h"
#include "utilmath.h"
#include "impl.h"
#include "assert.h"
#include "minalg.h"
#include "rigid_transform.h"

namespace tinygizmo
{


static minalg::float2 ring_points[] = {{+0.025f, 1}, {-0.025f, 1}, {-0.025f, 1}, {-0.025f, 1.1f}, {-0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1}};

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

static std::pair<const GizmoComponent *, float> raycast(const ray &ray)
{
    const GizmoComponent *updated_state = nullptr;
    float best_t = std::numeric_limits<float>::infinity();
    for (auto c : orientation_components)
    {
        auto f = intersect_ray_mesh(ray, c->mesh);
        if (f < best_t)
        {
            updated_state = c;
            best_t = f;
        }
    }
    return std::make_pair(updated_state, best_t);
}

bool orientation_gizmo(const gizmo_system &ctx, const std::string &name, fpalg::TRS &trs, bool is_local)
{
    auto &impl = ctx.m_impl;
    auto &t = fpalg::size_cast<rigid_transform>(trs);
    assert(length2(t.orientation) > float(1e-6));
    auto [gizmo, created] = impl->get_or_create_gizmo(hash_fnv1a(name));
    if (created)
    {
        // TODO
    }

    auto worldRay = impl->get_ray();
    rigid_transform gizmoTransform = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position); // Orientation is local by default
    auto localRay = detransform(gizmoTransform, worldRay);
    {
        auto [mesh, best_t] = raycast(localRay);

        // update
        if (impl->state.has_clicked)
        {
            if (mesh)
            {
                auto hit = localRay.origin + localRay.direction * best_t;
                auto worldHit = gizmoTransform.transform_point(hit);
                gizmo->begin(mesh, worldHit, t, {});
            }
        }
        else if (impl->state.has_released)
        {
            gizmo->end();
        }
    }

    // drag
    auto active = gizmo->activeMesh();
    if (active)
    {
        gizmo->axisRotationDragger(impl->state, worldRay, t.position, is_local, &gizmoTransform.orientation);
        if (!is_local)
        {
            t.orientation = qmul(gizmoTransform.orientation, gizmo->m_state.original.orientation);
        }
        else
        {
            t.orientation = gizmoTransform.orientation;
        }
    }

    // draw
    auto modelMatrix = fpalg::size_cast<minalg::float4x4>(gizmoTransform.matrix());
    {
        if (!is_local && active)
        {
            // For non-local transformations, we only present one rotation ring
            // and draw an arrow from the center of the gizmo to indicate the degree of rotation
            gizmo_renderable r{
                .mesh = active->mesh,
                .color = active->base_color,
            };
            for (auto &v : r.mesh.vertices)
            {
                v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
                v.normal = transform_vector(modelMatrix, v.normal);
            }
            impl->drawlist.push_back(r);

            {
                // Create orthonormal basis for drawing the arrow
                auto a = qrot(gizmoTransform.orientation, gizmo->m_state.originalPositionToClick());
                auto zDir = normalize(active->axis), xDir = normalize(cross(a, zDir)), yDir = cross(zDir, xDir);

                // Ad-hoc geometry
                minalg::float2 arrow_points[] = {{0.0f, 0.f}, {0.0f, 0.05f}, {0.8f, 0.05f}, {0.9f, 0.10f}, {1.0f, 0}};
                auto geo = geometry_mesh::make_lathed_geometry(yDir, xDir, zDir, 32, arrow_points, _countof(arrow_points));

                gizmo_renderable r;
                r.mesh = geo;
                r.color = minalg::float4(1);
                for (auto &v : r.mesh.vertices)
                {
                    v.position = transform_coord(modelMatrix, v.position);
                    v.normal = transform_vector(modelMatrix, v.normal);
                }
                impl->drawlist.push_back(r);
            }
        }
        else
        {
            for (auto mesh : orientation_components)
            {
                gizmo_renderable r{
                    .mesh = mesh->mesh,
                    .color = (mesh == active) ? mesh->base_color : mesh->highlight_color,
                };
                for (auto &v : r.mesh.vertices)
                {
                    v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
                    v.normal = transform_vector(modelMatrix, v.normal);
                }
                impl->drawlist.push_back(r);
            }
        }
    }

    return gizmo->isHoverOrActive();
}

} // namespace tinygizmo
