#include "tiny-gizmo.hpp"
#include "tiny-gizmo_interaction_state.h"
#include "utilmath.h"
#include "impl.h"
#include "assert.h"
#include "minalg.h"
#include "rigid_transform.h"

namespace tinygizmo
{

static const interact orientation_components[] = {
    interact::rotate_x,
    interact::rotate_y,
    interact::rotate_z,
};

bool orientation_gizmo(const gizmo_context &ctx, const std::string &name, rigid_transform &t, bool is_local)
{
    auto &impl = ctx.m_impl;
    {
        assert(length2(t.orientation) > float(1e-6));

        rigid_transform p = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position); // Orientation is local by default
        const float draw_scale = impl->get_gizmo_scale(p.position);
        const uint32_t id = hash_fnv1a(name);

        // interaction_mode will only change on clicked
        if (impl->state.has_clicked)
            impl->gizmos[id].interaction_mode = interact::none;

        interact updated_state = interact::none;

        auto ray = detransform(p, impl->get_ray());
        detransform(draw_scale, ray);
        float best_t = std::numeric_limits<float>::infinity();

        for (auto c : orientation_components)
        {
            auto f = intersect_ray_mesh(ray, impl->mesh_components[c].mesh);
            if (f < best_t)
            {
                updated_state = c;
                best_t = f;
            }
        }

        {
            if (impl->state.has_clicked)
            {
                impl->gizmos[id].interaction_mode = updated_state;
                if (impl->gizmos[id].interaction_mode != interact::none)
                {
                    transform(draw_scale, ray);
                    impl->gizmos[id].original_position = t.position;
                    impl->gizmos[id].original_orientation = t.orientation;
                    impl->gizmos[id].click_offset = p.transform_point(ray.origin + ray.direction * best_t);
                    impl->gizmos[id].active = true;
                }
                else
                    impl->gizmos[id].active = false;
            }
        }

        if (impl->gizmos[id].active)
        {
            p.orientation = impl->gizmos[id].rotation_dragger(impl->state, impl->get_ray(), t.position, is_local);
        }

        if (impl->state.has_released)
        {
            impl->gizmos[id].interaction_mode = interact::none;
            impl->gizmos[id].active = false;
        }

        auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());
        auto scaleMatrix = scaling_matrix(minalg::float3(draw_scale));
        modelMatrix = mul(modelMatrix, scaleMatrix);

        std::vector<interact> draw_interactions;
        if (!is_local && impl->gizmos[id].interaction_mode != interact::none)
            draw_interactions = {impl->gizmos[id].interaction_mode};
        else
            draw_interactions = {interact::rotate_x, interact::rotate_y, interact::rotate_z};

        for (auto c : draw_interactions)
        {
            gizmo_renderable r;
            r.mesh = impl->mesh_components[c].mesh;
            r.color = (c == impl->gizmos[id].interaction_mode) ? impl->mesh_components[c].base_color : impl->mesh_components[c].highlight_color;
            for (auto &v : r.mesh.vertices)
            {
                v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
                v.normal = transform_vector(modelMatrix, v.normal);
            }
            impl->drawlist.push_back(r);
        }

        // For non-local transformations, we only present one rotation ring
        // and draw an arrow from the center of the gizmo to indicate the degree of rotation
        if (is_local == false && impl->gizmos[id].interaction_mode != interact::none)
        {
            minalg::float3 activeAxis;
            switch (impl->gizmos[id].interaction_mode)
            {
            case interact::rotate_x:
                activeAxis = {1, 0, 0};
                break;
            case interact::rotate_y:
                activeAxis = {0, 1, 0};
                break;
            case interact::rotate_z:
                activeAxis = {0, 0, 1};
                break;
            }

            interaction_state &interaction = impl->gizmos[id];

            // Create orthonormal basis for drawing the arrow
            auto a = qrot(p.orientation, interaction.click_offset - interaction.original_position);
            auto zDir = normalize(activeAxis), xDir = normalize(cross(a, zDir)), yDir = cross(zDir, xDir);

            // Ad-hoc geometry
            std::initializer_list<minalg::float2> arrow_points = {{0.0f, 0.f}, {0.0f, 0.05f}, {0.8f, 0.05f}, {0.9f, 0.10f}, {1.0f, 0}};
            auto geo = geometry_mesh::make_lathed_geometry(yDir, xDir, zDir, 32, arrow_points);

            gizmo_renderable r;
            r.mesh = geo;
            r.color = minalg::float4(1);
            for (auto &v : r.mesh.vertices)
            {
                v.position = transform_coord(modelMatrix, v.position);
                v.normal = transform_vector(modelMatrix, v.normal);
            }
            impl->drawlist.push_back(r);

            t.orientation = qmul(p.orientation, interaction.original_orientation);
        }
        else if (is_local == true && impl->gizmos[id].interaction_mode != interact::none)
            t.orientation = p.orientation;

        return (impl->gizmos[id].hover || impl->gizmos[id].active);
    }
}

} // namespace tinygizmo
