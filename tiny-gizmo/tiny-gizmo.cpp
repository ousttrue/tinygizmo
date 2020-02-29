// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org>

#include "tiny-gizmo.hpp"
#include "geometry_mesh.h"
#include "rigid_transform.h"
#include "tiny-gizmo_interaction_state.h"

#include <assert.h>
#include <memory>
#include <vector>
#include <iostream>
#include <functional>
#include <map>
#include <string>
#include <chrono>
#include <castalg.h>
#include "utilmath.h"

using namespace minalg;

namespace tinygizmo
{
struct gizmo_mesh_component
{
    geometry_mesh mesh;
    float4 base_color, highlight_color;
};
struct gizmo_renderable
{
    geometry_mesh mesh;
    float4 color;
};

//////////////////////////////////
// Gizmo Context Implementation //
//////////////////////////////////

struct gizmo_context_impl
{
private:
    tinygizmo::geometry_mesh m_r{};

public:
    std::map<interact, gizmo_mesh_component> mesh_components;
    std::vector<gizmo_renderable> drawlist;

    std::map<uint32_t, interaction_state> gizmos;

    gizmo_application_state state;

    // world-space ray origin (i.e. the camera position)
    minalg::float3 ray_origin;
    // world-space ray direction
    minalg::float3 ray_direction;

    ray get_ray() const
    {
        return {ray_origin, ray_direction};
    }

    gizmo_context_impl()
    {
        std::vector<float2> arrow_points = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.10f}, {1.2f, 0}};
        std::vector<float2> mace_points = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.1f}, {1.25f, 0.1f}, {1.25f, 0}};
        std::vector<float2> ring_points = {{+0.025f, 1}, {-0.025f, 1}, {-0.025f, 1}, {-0.025f, 1.1f}, {-0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1}};
        mesh_components[interact::translate_x] = {geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, arrow_points), {1, 0.5f, 0.5f, 1.f}, {1, 0, 0, 1.f}};
        mesh_components[interact::translate_y] = {geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, arrow_points), {0.5f, 1, 0.5f, 1.f}, {0, 1, 0, 1.f}};
        mesh_components[interact::translate_z] = {geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, arrow_points), {0.5f, 0.5f, 1, 1.f}, {0, 0, 1, 1.f}};
        mesh_components[interact::translate_yz] = {geometry_mesh::make_box_geometry({-0.01f, 0.25, 0.25}, {0.01f, 0.75f, 0.75f}), {0.5f, 1, 1, 0.5f}, {0, 1, 1, 0.6f}};
        mesh_components[interact::translate_zx] = {geometry_mesh::make_box_geometry({0.25, -0.01f, 0.25}, {0.75f, 0.01f, 0.75f}), {1, 0.5f, 1, 0.5f}, {1, 0, 1, 0.6f}};
        mesh_components[interact::translate_xy] = {geometry_mesh::make_box_geometry({0.25, 0.25, -0.01f}, {0.75f, 0.75f, 0.01f}), {1, 1, 0.5f, 0.5f}, {1, 1, 0, 0.6f}};
        mesh_components[interact::translate_xyz] = {geometry_mesh::make_box_geometry({-0.05f, -0.05f, -0.05f}, {0.05f, 0.05f, 0.05f}), {0.9f, 0.9f, 0.9f, 0.25f}, {1, 1, 1, 0.35f}};
        mesh_components[interact::rotate_x] = {geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 32, ring_points, 0.003f), {1, 0.5f, 0.5f, 1.f}, {1, 0, 0, 1.f}};
        mesh_components[interact::rotate_y] = {geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 32, ring_points, -0.003f), {0.5f, 1, 0.5f, 1.f}, {0, 1, 0, 1.f}};
        mesh_components[interact::rotate_z] = {geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 32, ring_points), {0.5f, 0.5f, 1, 1.f}, {0, 0, 1, 1.f}};
        mesh_components[interact::scale_x] = {geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, mace_points), {1, 0.5f, 0.5f, 1.f}, {1, 0, 0, 1.f}};
        mesh_components[interact::scale_y] = {geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, mace_points), {0.5f, 1, 0.5f, 1.f}, {0, 1, 0, 1.f}};
        mesh_components[interact::scale_z] = {geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, mace_points), {0.5f, 0.5f, 1, 1.f}, {0, 0, 1, 1.f}};
    }

    // Returns a world-space ray through the given pixel, originating at the camera
    float3 get_ray_direction(int _x, int _y, int w, int h, const float4x4 &viewProjMatrix) const
    {
        const float x = 2 * (float)_x / w - 1;
        const float y = 1 - 2 * (float)_y / h;
        auto aspect_ratio = w / (float)h;
        const float4x4 inv_view_proj = inverse(viewProjMatrix);
        const float4 p0 = mul(inv_view_proj, float4(x, y, -1, 1)), p1 = mul(inv_view_proj, float4(x, y, +1, 1));
        return (p1.xyz() * p0.w - p0.xyz() * p1.w);
    }

    // Public methods
    void gizmo_context_impl::update(const gizmo_application_state &state, const std::array<float, 16> &view, const std::array<float, 16> &projection)
    {
        this->state = state;
        drawlist.clear();

        auto inv = inverse(castalg::ref_cast<minalg::float4x4>(view));
        // position[0] = inv.w.x;
        // position[1] = inv.w.y;
        // position[2] = inv.w.z;
        ray_origin = inv.w.xyz();

        auto m = mul(
            castalg::ref_cast<float4x4>(projection),
            castalg::ref_cast<float4x4>(view));

        ray_direction = get_ray_direction(
            state.mouse_x, state.mouse_y, state.window_width, state.window_height,
            m);
    }

    float get_gizmo_scale(const float3 &position) const
    {
        if (state.screenspace_scale <= 0.0f)
        {
            return 1.0f;
        }

        float dist = length(position - castalg::ref_cast<minalg::float3>(state.cam.position));
        return std::tan(state.cam.yfov) * dist * (state.screenspace_scale / state.viewport_size[1]);
    }

    const geometry_mesh &render()
    {
        m_r.clear();

        // Combine all gizmo sub-meshes into one super-mesh
        for (auto &m : drawlist)
        {
            uint32_t numVerts = (uint32_t)m_r.vertices.size();
            auto it = m_r.vertices.insert(m_r.vertices.end(), m.mesh.vertices.begin(), m.mesh.vertices.end());
            for (auto &f : m.mesh.triangles)
                m_r.triangles.push_back({numVerts + f.x, numVerts + f.y, numVerts + f.z});
            for (; it != m_r.vertices.end(); ++it)
                it->color = m.color; // Take the color and shove it into a per-vertex attribute
        }

        return m_r;
    }
};

///////////////////////////////
//   Gizmo Implementations   //
///////////////////////////////

gizmo_context::gizmo_context()
    : m_impl(new gizmo_context_impl)
{
}

gizmo_context::~gizmo_context()
{
    delete m_impl;
}

void gizmo_context::new_frame(const gizmo_application_state &state,
                              const std::array<float, 16> &view, const std::array<float, 16> &projection)
{
    m_impl->update(state, view, projection);
}

void gizmo_context::render(
    void **pVertices, uint32_t *veticesBytes, uint32_t *vertexStride,
    void **pIndices, uint32_t *indicesBytes, uint32_t *indexStride)
{
    auto &r = m_impl->render();
    *pVertices = (void *)r.vertices.data();
    *veticesBytes = static_cast<uint32_t>(r.vertices.size() * sizeof(r.vertices[0]));
    *vertexStride = sizeof(r.vertices[0]);
    *pIndices = (void *)r.triangles.data();
    *indicesBytes = static_cast<uint32_t>(r.triangles.size() * sizeof(r.triangles[0]));
    *indexStride = sizeof(r.triangles[0]) / 3;
}

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
    rigid_transform p = rigid_transform(is_local ? t.orientation : float4(0, 0, 0, 1), t.position);

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

        std::vector<float3> axes;
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
                self->plane_translation_dragger(impl->state, impl->get_ray(), -minalg::qzdir(castalg::ref_cast<minalg::float4>(impl->state.cam.orientation)), t.position);
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

        float4x4 modelMatrix = castalg::ref_cast<float4x4>(p.matrix());
        float4x4 scaleMatrix = scaling_matrix(float3(draw_scale));
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

        rigid_transform p = rigid_transform(is_local ? t.orientation : float4(0, 0, 0, 1), t.position); // Orientation is local by default
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

        float4x4 modelMatrix = castalg::ref_cast<float4x4>(p.matrix());
        float4x4 scaleMatrix = scaling_matrix(float3(draw_scale));
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
            float3 activeAxis;
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
            float3 a = qrot(p.orientation, interaction.click_offset - interaction.original_position);
            float3 zDir = normalize(activeAxis), xDir = normalize(cross(a, zDir)), yDir = cross(zDir, xDir);

            // Ad-hoc geometry
            std::initializer_list<float2> arrow_points = {{0.0f, 0.f}, {0.0f, 0.05f}, {0.8f, 0.05f}, {0.9f, 0.10f}, {1.0f, 0}};
            auto geo = geometry_mesh::make_lathed_geometry(yDir, xDir, zDir, 32, arrow_points);

            gizmo_renderable r;
            r.mesh = geo;
            r.color = float4(1);
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

        float4x4 modelMatrix = castalg::ref_cast<float4x4>(p.matrix());
        float4x4 scaleMatrix = scaling_matrix(float3(draw_scale));
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
