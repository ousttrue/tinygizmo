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

    virtual bool axisRotationDragger(
        const ray &ray, const GizmoState &state,
        bool is_local, float snap_rotation,
        rigid_transform *out) const
    {
        return false;
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
    // Flag to indicate if the gizmo is being hovered
    bool m_hover = false;
    // Currently active component
    const GizmoComponent *m_activeMesh = nullptr;

public:
    GizmoState m_state;

    bool isHoverOrActive() const { return m_hover || m_activeMesh; }
    void hover(bool enable) { m_hover = enable; }
    const GizmoComponent *activeMesh() const { return m_activeMesh; }

    void end()
    {
        m_activeMesh = nullptr;
    }

    void begin(const GizmoComponent *pMesh, const minalg::float3 &click, const rigid_transform &t, const minalg::float3 &axis)
    {
        m_activeMesh = pMesh;
        m_state = {
            .click = click,
            .original = t,
            .axis = axis,
        };
    }
};

} // namespace tinygizmo
