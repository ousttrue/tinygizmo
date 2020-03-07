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

struct GizmoComponent
{
    geometry_mesh mesh;
    minalg::float4 base_color;
    minalg::float4 highlight_color;
    minalg::float3 axis;

public:
    GizmoComponent(const geometry_mesh &_mesh,
                   const minalg::float4 &_base_color, const minalg::float4 &_highlight_color,
                   const minalg::float3 &_axis)
        : mesh(_mesh), base_color(_base_color), highlight_color(_highlight_color), axis(_axis)
    {
    }

    virtual bool dragger(
        const ray &ray, const GizmoState &state,
        const bool uniform,
        rigid_transform *t) const
    {
        return false;
    }

    virtual void translationDragger(const ray &r, const GizmoState &state,
                         const minalg::float3 &plane_normal, float snapValue,
                         minalg::float3 &point) const
    {
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
    const GizmoComponent *m_mesh = nullptr;

public:
    GizmoState m_state;

    bool isHoverOrActive() const { return m_hover || m_active; }
    void hover(bool enable) { m_hover = enable; }
    const GizmoComponent *mesh() const { return m_mesh; }

    void end()
    {
        m_active = false;
        m_mesh = nullptr;
    }

    void begin(const GizmoComponent *pMesh, const minalg::float3 &click, const rigid_transform &t, const minalg::float3 &axis)
    {
        m_active = true;
        m_mesh = pMesh;
        m_state = {
            .click = click,
            .original = t,
            .axis = axis,
        };
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