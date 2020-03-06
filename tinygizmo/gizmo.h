#include "geometry_mesh.h"
#include "functional"
#include "tinygizmo.h"
#include "utilmath.h"
#include <vector>

namespace tinygizmo
{

struct GizmoState
{
    // Offset from position of grabbed object to coordinates of clicked point
    minalg::float3 click;
    rigid_transform original;
    minalg::float3 axis;

    minalg::float3 originalPositionToClick() const
    {
        return click - original.position;
    }
};

struct gizmo_mesh_component
{
    geometry_mesh mesh;
    minalg::float4 base_color;
    minalg::float4 highlight_color;
    minalg::float3 axis;

    std::function<void(class Gizmo &gizmo,
                       const gizmo_application_state &state, const ray &r, const minalg::float3 &plane_normal, minalg::float3 &point)>
        dragger;

    bool axisScaleDragger(
        const ray &ray, const GizmoState &state,
        const bool uniform,
        minalg::float3 *scale)
    {
        // auto axis = m_mesh->axis;
        auto plane_tangent = cross(axis, state.original.position - fpalg::size_cast<minalg::float3>(ray.origin));
        auto plane_normal = cross(axis, plane_tangent);

        // If an intersection exists between the ray and the plane, place the object at that point
        const float denom = dot(ray.direction, plane_normal);
        if (std::abs(denom) == 0)
        {
            return false;
        }

        const float t = dot(state.original.position - ray.origin, plane_normal) / denom;
        if (t < 0)
        {
            return false;
        }

        auto distance = ray.origin + ray.direction * t;

        auto hoge = (distance - state.click);
        auto offset_on_axis = hoge * axis;
        flush_to_zero(offset_on_axis);
        // std::cout << offset_on_axis << std::endl;
        auto new_scale = state.original.scale + offset_on_axis;

        if (uniform)
            *scale = minalg::float3(clamp(dot(distance, new_scale), 0.01f, 1000.f));
        else
            *scale = minalg::float3(clamp(new_scale.x, 0.01f, 1000.f), clamp(new_scale.y, 0.01f, 1000.f), clamp(new_scale.z, 0.01f, 1000.f));
        return true;
    }
};

struct gizmo_renderable
{
    geometry_mesh mesh;
    minalg::float4 color;
};

class Gizmo
{
protected:
    // Flag to indicate if the gizmo is being actively manipulated
    bool m_active = false;
    // Flag to indicate if the gizmo is being hovered
    bool m_hover = false;
    // Currently active component
    gizmo_mesh_component *m_mesh = nullptr;

public:
    GizmoState m_state;

    bool isHoverOrActive() const { return m_hover || m_active; }
    void hover(bool enable) { m_hover = enable; }
    gizmo_mesh_component *mesh() { return m_mesh; }

    void end()
    {
        m_active = false;
        m_mesh = nullptr;
    }

    void begin(gizmo_mesh_component *pMesh, const minalg::float3 &click, const rigid_transform &t, const minalg::float3 &axis)
    {
        m_active = true;
        m_mesh = pMesh;
        m_state = {
            .click = click,
            .original = t,
            .axis = axis,
        };
    }

    void translationDragger(const gizmo_application_state &state, const ray &ray, minalg::float3 &position)
    {
        if (!m_active)
        {
            return;
        }

        position += m_state.click;
        m_mesh->dragger(*this, state, ray, m_state.axis, position);
        position -= m_state.click;
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
        auto start_orientation = is_local ? m_state.original.orientation : minalg::float4(0, 0, 0, 1);

        auto axis = m_mesh->axis;
        rigid_transform original_pose = {start_orientation, m_state.original.position};
        auto the_axis = original_pose.transform_vector(axis);
        minalg::float4 the_plane = {the_axis, -dot(the_axis, m_state.click)};

        float t;
        if (!intersect_ray_plane(r, the_plane, &t))
        {
            *out = start_orientation;
            return;
        }

        auto center_of_rotation = m_state.original.position + the_axis * dot(the_axis, m_state.originalPositionToClick());
        auto arm1 = normalize(m_state.click - center_of_rotation);
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

    virtual void onClick(const ray &ray, const rigid_transform &t) {}

    virtual void draw(const fpalg::Transform &t, std::vector<gizmo_renderable> &drawlist) {}
};

} // namespace tinygizmo