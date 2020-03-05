#pragma once
#include "minalg.h"
#include "geometry_mesh.h"
#include <unordered_map>
#include <functional>
#include <fpalg.h>

namespace tinygizmo
{

struct gizmo_mesh_component
{
    geometry_mesh mesh;
    minalg::float4 base_color;
    minalg::float4 highlight_color;
    minalg::float3 axis;

    std::function<void(struct interaction_state &gizmo,
                       const gizmo_application_state &state, const ray &r, const minalg::float3 &plane_normal, minalg::float3 &point)>
        dragger;
};

struct interaction_state
{
private:
    // Flag to indicate if the gizmo is being actively manipulated
    bool m_active = false;
    // Flag to indicate if the gizmo is being hovered
    bool m_hover = false;

public:
    bool isHoverOrActive() const { return m_hover || m_active; }
    void hover(bool enable) { m_hover = enable; }
    bool isActive() const { return m_active; }

    minalg::float3 original_position;     // Original position of an object being manipulated with a gizmo
    minalg::float4 original_orientation;  // Original orientation of an object being manipulated with a gizmo
    minalg::float3 original_scale;        // Original scale of an object being manipulated with a gizmo
    minalg::float3 click_offset;          // Offset from position of grabbed object to coordinates of clicked point
    gizmo_mesh_component *mesh = nullptr; // Currently active component
    minalg::float3 axis;

    void end()
    {
        m_active = false;
        mesh = nullptr;
    }

    void beginTranslation(gizmo_mesh_component *pMesh, const minalg::float3 &offset, const minalg::float3 &axis)
    {
        m_active = true;
        mesh = pMesh;
        click_offset = offset;
        this->axis = axis;
    }

    void beginRotation(gizmo_mesh_component *pMesh, const minalg::float3 &offset, const minalg::float3 &position, const minalg::float4 rotation)
    {
        m_active = true;
        mesh = pMesh;
        click_offset = offset;
        original_position = position;
        original_orientation = rotation;
    }

    void beginScale(gizmo_mesh_component *pMesh, const minalg::float3 &offset, const minalg::float3 &scale)
    {
        m_active = true;
        mesh = pMesh;
        click_offset = offset;
        original_scale = scale;
    }

    void axisScaleDragger(
        const gizmo_application_state &state, const ray &ray,
        const minalg::float3 &center, const bool uniform,
        minalg::float3 *scale)
    {
        if (!m_active)
        {
            return;
        }

        auto axis = mesh->axis;
        auto plane_tangent = cross(axis, center - fpalg::size_cast<minalg::float3>(state.camera_position));
        auto plane_normal = cross(axis, plane_tangent);

        // If an intersection exists between the ray and the plane, place the object at that point
        const float denom = dot(ray.direction, plane_normal);
        if (std::abs(denom) == 0)
        {
            return;
        }

        const float t = dot(center - ray.origin, plane_normal) / denom;
        if (t < 0)
        {
            return;
        }

        auto distance = ray.origin + ray.direction * t;

        auto hoge = (distance - click_offset);
        auto offset_on_axis = hoge * axis;
        flush_to_zero(offset_on_axis);
        // std::cout << offset_on_axis << std::endl;
        auto new_scale = original_scale + offset_on_axis;

        if (uniform)
            *scale = minalg::float3(clamp(dot(distance, new_scale), 0.01f, 1000.f));
        else
            *scale = minalg::float3(clamp(new_scale.x, 0.01f, 1000.f), clamp(new_scale.y, 0.01f, 1000.f), clamp(new_scale.z, 0.01f, 1000.f));
        if (state.snap_scale)
            *scale = snap(*scale, state.snap_scale);
    }
};

struct gizmo_renderable
{
    geometry_mesh mesh;
    minalg::float4 color;
};

struct gizmo_system_impl
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
    void gizmo_system_impl::update(const gizmo_application_state &state, const std::array<float, 16> &m)
    {
        this->state = state;
        drawlist.clear();

        ray_origin = fpalg::size_cast<minalg::float3>(state.camera_position);

        ray_direction = get_ray_direction(
            state.mouse_x, state.mouse_y, state.window_width, state.window_height,
            fpalg::size_cast<minalg::float4x4>(m));
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
