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

using namespace tinygizmo;
using namespace minalg;

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

static ray get_ray(const gizmo_application_state &state)
{
    return {
        castalg::ref_cast<minalg::float3>(state.ray_origin),
        castalg::ref_cast<minalg::float3>(state.ray_direction)};
}

struct gizmo_context::gizmo_context_impl
{
private:
    tinygizmo::geometry_mesh m_r{};

public:
    std::map<interact, gizmo_mesh_component> mesh_components;
    std::vector<gizmo_renderable> drawlist;

    std::map<uint32_t, interaction_state> gizmos;

    gizmo_application_state state;

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

    // Public methods
    void gizmo_context_impl::update(const gizmo_application_state &state)
    {
        this->state = state;

        drawlist.clear();
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

// This will calculate a scale constant based on the number of screenspace pixels passed as pixel_scale.
static float scale_screenspace(const camera_parameters &cam, const float3 position, const float pixel_scale, uint32_t viewport_height)
{
    float dist = length(position - castalg::ref_cast<minalg::float3>(cam.position));
    return std::tan(cam.yfov) * dist * (pixel_scale / viewport_height);
}

// The only purpose of this is readability: to reduce the total column width of the intersect(...) statements in every gizmo
bool intersect(gizmo_context::gizmo_context_impl &g, const ray &r, interact i, float &t, const float best_t)
{
    if (intersect_ray_mesh(r, g.mesh_components[i].mesh, &t) && t < best_t)
        return true;
    return false;
}

///////////////////////////////
//   Gizmo Implementations   //
///////////////////////////////

interaction_state &orientation_gizmo(const std::string &name, bool is_local, gizmo_context::gizmo_context_impl &g, const float3 &center, float4 &orientation)
{
    assert(length2(orientation) > float(1e-6));

    rigid_transform p = rigid_transform(is_local ? orientation : float4(0, 0, 0, 1), center); // Orientation is local by default
    const float draw_scale = (g.state.screenspace_scale > 0.f) ? scale_screenspace(g.state.cam, p.position, g.state.screenspace_scale, g.state.viewport_size[1]) : 1.f;
    const uint32_t id = hash_fnv1a(name);

    // interaction_mode will only change on clicked
    if (g.state.has_clicked)
        g.gizmos[id].interaction_mode = interact::none;

    {
        interact updated_state = interact::none;

        auto ray = detransform(p, get_ray(g.state));
        detransform(draw_scale, ray);
        float best_t = std::numeric_limits<float>::infinity(), t;

        if (intersect(g, ray, interact::rotate_x, t, best_t))
        {
            updated_state = interact::rotate_x;
            best_t = t;
        }
        if (intersect(g, ray, interact::rotate_y, t, best_t))
        {
            updated_state = interact::rotate_y;
            best_t = t;
        }
        if (intersect(g, ray, interact::rotate_z, t, best_t))
        {
            updated_state = interact::rotate_z;
            best_t = t;
        }

        if (g.state.has_clicked)
        {
            g.gizmos[id].interaction_mode = updated_state;
            if (g.gizmos[id].interaction_mode != interact::none)
            {
                transform(draw_scale, ray);
                g.gizmos[id].original_position = center;
                g.gizmos[id].original_orientation = orientation;
                g.gizmos[id].click_offset = p.transform_point(ray.origin + ray.direction * t);
                g.gizmos[id].active = true;
            }
            else
                g.gizmos[id].active = false;
        }
    }

    if (g.gizmos[id].active)
    {
        p.orientation = g.gizmos[id].rotation_dragger(g.state, center, is_local);
    }

    if (g.state.has_released)
    {
        g.gizmos[id].interaction_mode = interact::none;
        g.gizmos[id].active = false;
    }

    float4x4 modelMatrix = castalg::ref_cast<float4x4>(p.matrix());
    float4x4 scaleMatrix = scaling_matrix(float3(draw_scale));
    modelMatrix = mul(modelMatrix, scaleMatrix);

    std::vector<interact> draw_interactions;
    if (!is_local && g.gizmos[id].interaction_mode != interact::none)
        draw_interactions = {g.gizmos[id].interaction_mode};
    else
        draw_interactions = {interact::rotate_x, interact::rotate_y, interact::rotate_z};

    for (auto c : draw_interactions)
    {
        gizmo_renderable r;
        r.mesh = g.mesh_components[c].mesh;
        r.color = (c == g.gizmos[id].interaction_mode) ? g.mesh_components[c].base_color : g.mesh_components[c].highlight_color;
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        g.drawlist.push_back(r);
    }

    // For non-local transformations, we only present one rotation ring
    // and draw an arrow from the center of the gizmo to indicate the degree of rotation
    if (is_local == false && g.gizmos[id].interaction_mode != interact::none)
    {
        float3 activeAxis;
        switch (g.gizmos[id].interaction_mode)
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

        interaction_state &interaction = g.gizmos[id];

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
        g.drawlist.push_back(r);

        orientation = qmul(p.orientation, interaction.original_orientation);
    }
    else if (is_local == true && g.gizmos[id].interaction_mode != interact::none)
        orientation = p.orientation;

    return g.gizmos[id];
}

interaction_state &scale_gizmo(const std::string &name, gizmo_context::gizmo_context_impl &g, const float4 &orientation, const float3 &center, float3 &scale)
{
    rigid_transform p = rigid_transform(orientation, center);
    const float draw_scale = (g.state.screenspace_scale > 0.f) ? scale_screenspace(g.state.cam, p.position, g.state.screenspace_scale, g.state.viewport_size[1]) : 1.f;
    const uint32_t id = hash_fnv1a(name);

    if (g.state.has_clicked)
        g.gizmos[id].interaction_mode = interact::none;

    {
        interact updated_state = interact::none;
        auto ray = detransform(p, get_ray(g.state));
        detransform(draw_scale, ray);
        float best_t = std::numeric_limits<float>::infinity(), t;
        if (intersect(g, ray, interact::scale_x, t, best_t))
        {
            updated_state = interact::scale_x;
            best_t = t;
        }
        if (intersect(g, ray, interact::scale_y, t, best_t))
        {
            updated_state = interact::scale_y;
            best_t = t;
        }
        if (intersect(g, ray, interact::scale_z, t, best_t))
        {
            updated_state = interact::scale_z;
            best_t = t;
        }

        if (g.state.has_clicked)
        {
            g.gizmos[id].interaction_mode = updated_state;
            if (g.gizmos[id].interaction_mode != interact::none)
            {
                transform(draw_scale, ray);
                g.gizmos[id].original_scale = scale;
                g.gizmos[id].click_offset = p.transform_point(ray.origin + ray.direction * t);
                g.gizmos[id].active = true;
            }
            else
                g.gizmos[id].active = false;
        }
    }

    if (g.state.has_released)
    {
        g.gizmos[id].interaction_mode = interact::none;
        g.gizmos[id].active = false;
    }

    g.gizmos[id].scale_dragger(g.state, center, &scale, false);

    float4x4 modelMatrix = castalg::ref_cast<float4x4>(p.matrix());
    float4x4 scaleMatrix = scaling_matrix(float3(draw_scale));
    modelMatrix = mul(modelMatrix, scaleMatrix);

    std::vector<interact> draw_components{interact::scale_x, interact::scale_y, interact::scale_z};

    for (auto c : draw_components)
    {
        gizmo_renderable r;
        r.mesh = g.mesh_components[c].mesh;
        r.color = (c == g.gizmos[id].interaction_mode) ? g.mesh_components[c].base_color : g.mesh_components[c].highlight_color;
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        g.drawlist.push_back(r);
    }

    return g.gizmos[id];
}

std::array<float, 16> camera_parameters::get_view_projection_matrix(const std::array<float, 16> &view, const std::array<float, 16> &projection) const
{
    auto m = mul(
        castalg::ref_cast<float4x4>(projection),
        castalg::ref_cast<float4x4>(view));
    return castalg::ref_cast<std::array<float, 16>>(m);
}

// Returns a world-space ray through the given pixel, originating at the camera
std::array<float, 3> camera_parameters::get_ray_direction(int _x, int _y, int w, int h, const std::array<float, 16> &viewProjMatrix) const
{
    const float x = 2 * (float)_x / w - 1;
    const float y = 1 - 2 * (float)_y / h;
    auto aspect_ratio = w / (float)h;
    const float4x4 inv_view_proj = inverse(castalg::ref_cast<float4x4>(viewProjMatrix));
    const float4 p0 = mul(inv_view_proj, float4(x, y, -1, 1)), p1 = mul(inv_view_proj, float4(x, y, +1, 1));
    return castalg::ref_cast<std::array<float, 3>>(p1.xyz() * p0.w - p0.xyz() * p1.w);
}

//////////////////////////////////
// Public Gizmo Implementations //
//////////////////////////////////

gizmo_context::gizmo_context()
    : impl(new gizmo_context_impl)
{
}

gizmo_context::~gizmo_context()
{
}

void gizmo_context::new_frame(const gizmo_application_state &state)
{
    impl->update(state);
}

void gizmo_context::render(
    void **pVertices, uint32_t *veticesBytes, uint32_t *vertexStride,
    void **pIndices, uint32_t *indicesBytes, uint32_t *indexStride)
{
    auto &r = impl->render();
    *pVertices = (void *)r.vertices.data();
    *veticesBytes = static_cast<uint32_t>(r.vertices.size() * sizeof(r.vertices[0]));
    *vertexStride = sizeof(r.vertices[0]);
    *pIndices = (void *)r.triangles.data();
    *indicesBytes = static_cast<uint32_t>(r.triangles.size() * sizeof(r.triangles[0]));
    *indexStride = sizeof(r.triangles[0]) / 3;
}

bool tinygizmo::gizmo_context::position_gizmo(const std::string &name, rigid_transform &t, bool is_local)
{
    const uint32_t id = hash_fnv1a(name);
    auto self = &impl->gizmos[id];
    // ::position_gizmo(&s, is_local, *this->impl, t.orientation, t.position);
    // void position_gizmo(interaction_state *self, bool is_local, gizmo_context::gizmo_context_impl &g, const float4 &orientation, float3 &position)
    {
        rigid_transform p = rigid_transform(is_local ? t.orientation : float4(0, 0, 0, 1), t.position);
        const float draw_scale = (impl->state.screenspace_scale > 0.f) ? scale_screenspace(impl->state.cam, p.position, impl->state.screenspace_scale, impl->state.viewport_size[1]) : 1.f;

        // interaction_mode will only change on clicked
        if (impl->state.has_clicked)
            self->interaction_mode = interact::none;

        {
            interact updated_state = interact::none;
            auto ray = detransform(p, get_ray(impl->state));
            detransform(draw_scale, ray);

            float best_t = std::numeric_limits<float>::infinity(), t;
            if (intersect(*impl, ray, interact::translate_x, t, best_t))
            {
                updated_state = interact::translate_x;
                best_t = t;
            }
            if (intersect(*impl, ray, interact::translate_y, t, best_t))
            {
                updated_state = interact::translate_y;
                best_t = t;
            }
            if (intersect(*impl, ray, interact::translate_z, t, best_t))
            {
                updated_state = interact::translate_z;
                best_t = t;
            }
            if (intersect(*impl, ray, interact::translate_yz, t, best_t))
            {
                updated_state = interact::translate_yz;
                best_t = t;
            }
            if (intersect(*impl, ray, interact::translate_zx, t, best_t))
            {
                updated_state = interact::translate_zx;
                best_t = t;
            }
            if (intersect(*impl, ray, interact::translate_xy, t, best_t))
            {
                updated_state = interact::translate_xy;
                best_t = t;
            }
            if (intersect(*impl, ray, interact::translate_xyz, t, best_t))
            {
                updated_state = interact::translate_xyz;
                best_t = t;
            }

            if (impl->state.has_clicked)
            {
                self->interaction_mode = updated_state;

                if (self->interaction_mode != interact::none)
                {
                    transform(draw_scale, ray);
                    self->click_offset = is_local ? p.transform_vector(ray.origin + ray.direction * t) : ray.origin + ray.direction * t;
                    self->active = true;
                }
                else
                    self->active = false;
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
                self->axis_translation_dragger(impl->state, axes[0], t.position);
                break;
            case interact::translate_y:
                self->axis_translation_dragger(impl->state, axes[1], t.position);
                break;
            case interact::translate_z:
                self->axis_translation_dragger(impl->state, axes[2], t.position);
                break;
            case interact::translate_yz:
                self->plane_translation_dragger(impl->state, axes[0], t.position);
                break;
            case interact::translate_zx:
                self->plane_translation_dragger(impl->state, axes[1], t.position);
                break;
            case interact::translate_xy:
                self->plane_translation_dragger(impl->state, axes[2], t.position);
                break;
            case interact::translate_xyz:
                self->plane_translation_dragger(impl->state, -minalg::qzdir(castalg::ref_cast<minalg::float4>(impl->state.cam.orientation)), t.position);
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

bool tinygizmo::gizmo_context::orientation_gizmo(const std::string &name, rigid_transform &t, bool is_local)
{
    auto &s = ::orientation_gizmo(name, is_local, *this->impl, t.position, t.orientation);
    return (s.hover || s.active);
}

bool tinygizmo::gizmo_context::scale_gizmo(const std::string &name, rigid_transform &t)
{
    auto &s = ::scale_gizmo(name, *this->impl, t.orientation, t.position, t.scale);
    return (s.hover || s.active);
}
