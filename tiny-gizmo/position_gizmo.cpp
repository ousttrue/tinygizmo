#include "tiny-gizmo.hpp"
#include "tiny-gizmo_interaction_state.h"
#include "utilmath.h"
#include "minalg.h"
#include "impl.h"

namespace tinygizmo
{
static const interact translation_components[] = {
    interact::translate_x,
    interact::translate_y,
    interact::translate_z,
    interact::translate_xy,
    interact::translate_yz,
    interact::translate_zx,
    interact::translate_xyz,
};

bool position_gizmo(const gizmo_context &ctx, const std::string &name, rigid_transform &t, bool is_local)
{
    auto &impl = ctx.m_impl;
    const uint32_t id = hash_fnv1a(name);
    auto self = &impl->gizmos[id];
    rigid_transform p = rigid_transform(is_local ? t.orientation : minalg::float4(0, 0, 0, 1), t.position);

    {
        const float draw_scale = impl->get_gizmo_scale(t.position);

        // interaction_mode will only change on clicked
        if (impl->state.has_clicked)
            self->interaction_mode = interact::none;

        interact updated_state = interact::none;
        auto ray = detransform(p, impl->get_ray());
        detransform(draw_scale, ray);

        float best_t = std::numeric_limits<float>::infinity();
        for (auto c : translation_components)
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
                self->interaction_mode = updated_state;

                if (self->interaction_mode != interact::none)
                {
                    transform(draw_scale, ray);
                    self->click_offset = is_local ? p.transform_vector(ray.origin + ray.direction * best_t) : ray.origin + ray.direction * best_t;
                    self->active = true;
                }
                else
                {
                    self->active = false;
                }
            }

            self->hover = (best_t == std::numeric_limits<float>::infinity()) ? false : true;
        }

        std::vector<minalg::float3> axes;
        if (is_local)
            axes = {qxdir(p.orientation), qydir(p.orientation), qzdir(p.orientation)};
        else
            axes = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

        if (self->active)
        {
            t.position += self->click_offset;
            switch (self->interaction_mode)
            {
            case interact::translate_x:
                self->axis_translation_dragger(impl->state, impl->get_ray(), axes[0], t.position);
                break;
            case interact::translate_y:
                self->axis_translation_dragger(impl->state, impl->get_ray(), axes[1], t.position);
                break;
            case interact::translate_z:
                self->axis_translation_dragger(impl->state, impl->get_ray(), axes[2], t.position);
                break;
            case interact::translate_yz:
                self->plane_translation_dragger(impl->state, impl->get_ray(), axes[0], t.position);
                break;
            case interact::translate_zx:
                self->plane_translation_dragger(impl->state, impl->get_ray(), axes[1], t.position);
                break;
            case interact::translate_xy:
                self->plane_translation_dragger(impl->state, impl->get_ray(), axes[2], t.position);
                break;
            case interact::translate_xyz:
                self->plane_translation_dragger(impl->state, impl->get_ray(), -minalg::qzdir(castalg::ref_cast<minalg::float4>(impl->state.camera_orientation)), t.position);
                break;
            }
            t.position -= self->click_offset;
        }

        if (impl->state.has_released)
        {
            self->interaction_mode = interact::none;
            self->active = false;
        }

        std::vector<interact> draw_interactions{
            interact::translate_x, interact::translate_y, interact::translate_z,
            interact::translate_yz, interact::translate_zx, interact::translate_xy,
            interact::translate_xyz};

        auto modelMatrix = castalg::ref_cast<minalg::float4x4>(p.matrix());
        auto scaleMatrix = scaling_matrix(minalg::float3(draw_scale));
        modelMatrix = mul(modelMatrix, scaleMatrix);

        for (auto c : draw_interactions)
        {
            gizmo_renderable r;
            r.mesh = impl->mesh_components[c].mesh;
            r.color = (c == self->interaction_mode) ? impl->mesh_components[c].base_color : impl->mesh_components[c].highlight_color;
            for (auto &v : r.mesh.vertices)
            {
                v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
                v.normal = transform_vector(modelMatrix, v.normal);
            }
            impl->drawlist.push_back(r);
        }
    }

    return (self->hover || self->active);
}

} // namespace tinygizmo
