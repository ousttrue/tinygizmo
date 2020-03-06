#pragma once
#include "minalg.h"
#include "gizmo.h"
#include <unordered_map>
#include <functional>
#include <fpalg.h>

namespace tinygizmo
{

struct gizmo_system_impl
{
private:
    tinygizmo::geometry_mesh m_r{};
    std::unordered_map<uint32_t, std::unique_ptr<gizmo>> m_gizmos;

public:
    std::vector<gizmo_renderable> drawlist;

    std::pair<gizmo *, bool> get_or_create_gizmo(uint32_t id)
    {
        auto found = m_gizmos.find(id);
        bool created = false;
        if (found == m_gizmos.end())
        {
            // not found
            found = m_gizmos.insert(std::make_pair(id, std::make_unique<gizmo>())).first;
            created = true;
        }
        return std::make_pair(found->second.get(), created);
    }

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
    void gizmo_system_impl::update(const gizmo_application_state &state, const std::array<float, 16> &m)
    {
        this->state = state;
        drawlist.clear();

        ray_origin = fpalg::size_cast<minalg::float3>(state.camera_position);

        ray_direction = get_ray_direction(
            state.mouse_x, state.mouse_y, state.window_width, state.window_height,
            fpalg::size_cast<minalg::float4x4>(m));

        m_r.clear();
    }

    const geometry_mesh &render()
    {
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
