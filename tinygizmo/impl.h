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

struct gizmo_renderable
{
    geometry_mesh mesh;
    minalg::float4 color;
};

class interaction_state
{
    // Flag to indicate if the gizmo is being actively manipulated
    bool m_active = false;
    // Flag to indicate if the gizmo is being hovered
    bool m_hover = false;
    // Offset from position of grabbed object to coordinates of clicked point
    minalg::float3 m_click;
    // Currently active component
    gizmo_mesh_component *m_mesh = nullptr;
    std::list<gizmo_mesh_component *> m_meshes;

public:
    // Original position of an object being manipulated with a gizmo
    minalg::float3 m_original_position;

private:
    // Original orientation of an object being manipulated with a gizmo
    minalg::float4 m_original_orientation;
    // Original scale of an object being manipulated with a gizmo
    minalg::float3 m_original_scale;

    minalg::float3 m_axis;

public:
    bool isHoverOrActive() const { return m_hover || m_active; }
    void hover(bool enable) { m_hover = enable; }
    // bool isActive() const { return m_active; }
    void addMesh(gizmo_mesh_component *mesh) { m_meshes.push_back(mesh); }
    gizmo_mesh_component *mesh() { return m_mesh; }

    minalg::float4 originalOrientation() const
    {
        return m_original_orientation;
    }

    minalg::float3 originalPositionToClick() const
    {
        return m_click - m_original_position;
    }

    void end()
    {
        m_active = false;
        m_mesh = nullptr;
    }

    void beginTranslation(gizmo_mesh_component *pMesh, const minalg::float3 &click, const minalg::float3 &axis)
    {
        m_active = true;
        m_mesh = pMesh;
        m_click = click;
        m_axis = axis;
    }

    void translationDragger(const gizmo_application_state &state, const ray &ray, minalg::float3 &position)
    {
        if (!m_active)
        {
            return;
        }

        position += m_click;
        m_mesh->dragger(*this, state, ray, m_axis, position);
        position -= m_click;
    }

    void beginRotation(gizmo_mesh_component *pMesh, const minalg::float3 &click, const minalg::float3 &position, const minalg::float4 rotation)
    {
        m_active = true;
        m_mesh = pMesh;
        m_click = click;
        m_original_position = position;
        m_original_orientation = rotation;
    }

    void axisRotationDragger(
        const gizmo_application_state &state, const ray &r,
        const minalg::float3 &center, bool is_local,
        minalg::float4 *out)
    {
        if (!m_active)
        {
            return;
        }
        if (!state.mouse_left)
        {
            return;
        }
        auto start_orientation = is_local ? m_original_orientation : minalg::float4(0, 0, 0, 1);

        auto axis = m_mesh->axis;
        rigid_transform original_pose = {start_orientation, m_original_position};
        auto the_axis = original_pose.transform_vector(axis);
        minalg::float4 the_plane = {the_axis, -dot(the_axis, m_click)};

        float t;
        if (!intersect_ray_plane(r, the_plane, &t))
        {
            *out = start_orientation;
            return;
        }

        auto center_of_rotation = m_original_position + the_axis * dot(the_axis, m_click - m_original_position);
        auto arm1 = normalize(m_click - center_of_rotation);
        auto arm2 = normalize(r.origin + r.direction * t - center_of_rotation);

        float d = dot(arm1, arm2);
        if (d > 0.999f)
        {
            *out = start_orientation;
            return;
        }

        float angle = std::acos(d);
        if (angle < 0.001f)
        {
            *out = start_orientation;
            return;
        }

        if (state.snap_rotation)
        {
            auto snapped = make_rotation_quat_between_vectors_snapped(arm1, arm2, state.snap_rotation);
            *out = qmul(snapped, start_orientation);
            return;
        }
        else
        {
            auto a = normalize(cross(arm1, arm2));
            *out = qmul(rotation_quat(a, angle), start_orientation);
            return;
        }
    }

    void draw(const rigid_transform &t, std::vector<gizmo_renderable> &drawlist)
    {
        rigid_transform withoutScale(t.orientation, t.position);
        auto modelMatrix = fpalg::size_cast<minalg::float4x4>(withoutScale.matrix());
        for (auto mesh : m_meshes)
        {
            gizmo_renderable r{
                .mesh = mesh->mesh,
                .color = (mesh == m_mesh) ? mesh->base_color : mesh->highlight_color,
            };
            for (auto &v : r.mesh.vertices)
            {
                v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
                v.normal = transform_vector(modelMatrix, v.normal);
            }
            drawlist.push_back(r);
        }
    }

    void onClick(const ray &ray, const rigid_transform &t)
    {
        gizmo_mesh_component *updated_state = nullptr;
        float best_t = std::numeric_limits<float>::infinity();
        for (auto mesh : m_meshes)
        {
            auto t = intersect_ray_mesh(ray, mesh->mesh);
            if (t < best_t)
            {
                updated_state = mesh;
                best_t = t;
            }
        }

        if (updated_state)
        {
            rigid_transform withoutScale(t.orientation, t.position);
            beginScale(updated_state, withoutScale.transform_point(ray.origin + ray.direction * best_t), t.scale);
        }
    }

    void beginScale(gizmo_mesh_component *pMesh, const minalg::float3 &click, const minalg::float3 &scale)
    {
        m_active = true;
        m_mesh = pMesh;
        m_click = click;
        m_original_scale = scale;
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

        auto axis = m_mesh->axis;
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

        auto hoge = (distance - m_click);
        auto offset_on_axis = hoge * axis;
        flush_to_zero(offset_on_axis);
        // std::cout << offset_on_axis << std::endl;
        auto new_scale = m_original_scale + offset_on_axis;

        if (uniform)
            *scale = minalg::float3(clamp(dot(distance, new_scale), 0.01f, 1000.f));
        else
            *scale = minalg::float3(clamp(new_scale.x, 0.01f, 1000.f), clamp(new_scale.y, 0.01f, 1000.f), clamp(new_scale.z, 0.01f, 1000.f));
        if (state.snap_scale)
            *scale = snap(*scale, state.snap_scale);
    }
};

struct gizmo_system_impl
{
private:
    tinygizmo::geometry_mesh m_r{};
    std::unordered_map<uint32_t, std::unique_ptr<interaction_state>> m_gizmos;

public:
    std::vector<gizmo_renderable> drawlist;

    std::pair<interaction_state *, bool> get_or_create_gizmo(uint32_t id)
    {
        auto found = m_gizmos.find(id);
        bool created = false;
        if (found == m_gizmos.end())
        {
            // not found
            found = m_gizmos.insert(std::make_pair(id, std::make_unique<interaction_state>())).first;
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
