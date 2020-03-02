#pragma once
#include "minalg.h"
#include "geometry_mesh.h"
#include <castalg.h>
#include <unordered_map>
#include <functional>

namespace tinygizmo
{

struct gizmo_mesh_component
{
    geometry_mesh mesh;
    minalg::float4 base_color;
    minalg::float4 highlight_color;
    minalg::float3 axis;

    std::function<void(struct interaction_state &gizmo,
                         const gizmo_application_state &state, const ray &r, const minalg::float3 &plane_normal, minalg::float3 &point)> dragger;
};

struct interaction_state
{
    bool active{false};                   // Flag to indicate if the gizmo is being actively manipulated
    bool hover{false};                    // Flag to indicate if the gizmo is being hovered
    minalg::float3 original_position;     // Original position of an object being manipulated with a gizmo
    minalg::float4 original_orientation;  // Original orientation of an object being manipulated with a gizmo
    minalg::float3 original_scale;        // Original scale of an object being manipulated with a gizmo
    minalg::float3 click_offset;          // Offset from position of grabbed object to coordinates of clicked point
    gizmo_mesh_component *mesh = nullptr; // Currently active component
    minalg::float3 axis;
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
