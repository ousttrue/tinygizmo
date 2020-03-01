#include "tiny-gizmo.hpp"
#include "tiny-gizmo_interaction_state.h"
#include "utilmath.h"
#include "impl.h"

namespace tinygizmo
{

static const interact scale_components[] = {
    interact::scale_x,
    interact::scale_y,
    interact::scale_z,
};

bool scale_gizmo(const gizmo_context &ctx, const std::string &name, rigid_transform &t)
{
    auto &impl = ctx.m_impl;
    // auto &s = ::scale_gizmo(name, *this->impl, t.orientation, t.position, t.scale);
    auto &scale = t.scale;
    // interaction_state &scale_gizmo(const std::string &name, gizmo_context::gizmo_context_impl &g, const float4 &orientation, const float3 &center, float3 &scale)
    {
        rigid_transform p = rigid_transform(t.orientation, t.position);
        const float draw_scale = impl->get_gizmo_scale(p.position);
        const uint32_t id = hash_fnv1a(name);

        if (impl->state.has_clicked)
            impl->gizmos[id].interaction_mode = interact::none;

        interact updated_state = interact::none;
        auto ray = detransform(p, impl->get_ray());
        detransform(draw_scale, ray);
        float best_t = std::numeric_limits<float>::infinity();

        for (auto c : scale_components)
        {
            auto t = intersect_ray_mesh(ray, impl->mesh_components[c].mesh);
            if (t < best_t)
            {
                updated_state = c;
                best_t = t;
            }
        }

        {
            if (impl->state.has_clicked)
            {
                impl->gizmos[id].interaction_mode = updated_state;
                if (impl->gizmos[id].interaction_mode != interact::none)
                {
                    transform(draw_scale, ray);
                    impl->gizmos[id].original_scale = scale;
                    impl->gizmos[id].click_offset = p.transform_point(ray.origin + ray.direction * best_t);
                    impl->gizmos[id].active = true;
                }
                else
                    impl->gizmos[id].active = false;
            }
        }

        if (impl->state.has_released)
        {
            impl->gizmos[id].interaction_mode = interact::none;
            impl->gizmos[id].active = false;
        }

        impl->gizmos[id].scale_dragger(impl->state, impl->get_ray(), t.position, &scale, false);

        auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());
        auto scaleMatrix = scaling_matrix(minalg::float3(draw_scale));
        modelMatrix = mul(modelMatrix, scaleMatrix);

        std::vector<interact> draw_components{interact::scale_x, interact::scale_y, interact::scale_z};

        for (auto c : draw_components)
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

        // return impl->gizmos[id];
        return (impl->gizmos[id].hover || impl->gizmos[id].active);
    }
}

} // namespace tinygizmo
