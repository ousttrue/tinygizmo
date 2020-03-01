#pragma once
#include "minalg.h"
#include "geometry_mesh.h"
#include <castalg.h>
#include <unordered_map>

namespace tinygizmo
{

struct gizmo_mesh_component
{
    geometry_mesh mesh;
    minalg::float4 base_color, highlight_color;
};

struct gizmo_renderable
{
    geometry_mesh mesh;
    minalg::float4 color;
};

struct gizmo_context_impl
{
private:
    tinygizmo::geometry_mesh m_r{};

public:
    std::unordered_map<interact, gizmo_mesh_component> mesh_components;
    std::vector<gizmo_renderable> drawlist;

    std::unordered_map<uint32_t, interaction_state> gizmos;

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
        std::vector<minalg::float2> mace_points = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.1f}, {1.25f, 0.1f}, {1.25f, 0}};
        mesh_components[interact::scale_x] = {geometry_mesh::make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, mace_points), {1, 0.5f, 0.5f, 1.f}, {1, 0, 0, 1.f}};
        mesh_components[interact::scale_y] = {geometry_mesh::make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, mace_points), {0.5f, 1, 0.5f, 1.f}, {0, 1, 0, 1.f}};
        mesh_components[interact::scale_z] = {geometry_mesh::make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, mace_points), {0.5f, 0.5f, 1, 1.f}, {0, 0, 1, 1.f}};
    }

    // Returns a world-space ray through the given pixel, originating at the camera
    minalg::float3 get_ray_direction(int _x, int _y, int w, int h, const minalg::float4x4 &viewProjMatrix) const
    {
        const float x = 2 * (float)_x / w - 1;
        const float y = 1 - 2 * (float)_y / h;
        auto aspect_ratio = w / (float)h;
        auto inv_view_proj = inverse(viewProjMatrix);
        auto p0 = mul(inv_view_proj, minalg::float4(x, y, -1, 1));
        auto p1 = mul(inv_view_proj, minalg::float4(x, y, +1, 1));
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
            castalg::ref_cast<minalg::float4x4>(projection),
            castalg::ref_cast<minalg::float4x4>(view));

        ray_direction = get_ray_direction(
            state.mouse_x, state.mouse_y, state.window_width, state.window_height,
            m);
    }

    float get_gizmo_scale(const minalg::float3 &position) const
    {
        if (state.screenspace_scale <= 0.0f)
        {
            return 1.0f;
        }

        // float dist = length(position - castalg::ref_cast<minalg::float3>(state.cam.position));
        // return std::tan(state.cam.yfov) * dist * (state.screenspace_scale / state.viewport_size[1]);
        return 1.0f;
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

} // namespace  tinygizmo
