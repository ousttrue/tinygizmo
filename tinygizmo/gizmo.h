#include "geometry_mesh.h"
#include <vector>

namespace tinygizmo
{

struct GizmoState
{
    // Offset from position of grabbed object to coordinates of clicked point
    fpalg::TRS original;
    fpalg::float3 offset;
    fpalg::float3 axis;
};

struct GizmoComponent
{
    geometry_mesh mesh;
    fpalg::float4 base_color;
    fpalg::float4 highlight_color;
    fpalg::float3 axis;
};

class Gizmo
{
protected:
    // Flag to indicate if the gizmo is being hovered
    bool m_hover = false;
    // Currently active component
    const GizmoComponent *m_active = nullptr;

public:
    GizmoState m_state;

    bool isHoverOrActive() const { return m_hover || m_active; }
    void hover(bool enable) { m_hover = enable; }
    const GizmoComponent *active() const { return m_active; }

    void end()
    {
        m_active = nullptr;
    }

    void begin(const GizmoComponent *pMesh, const fpalg::float3 &offset, const fpalg::TRS &t, const fpalg::float3 &axis)
    {
        m_active = pMesh;
        m_state = {
            .original = t,
            .offset = offset,
            .axis = axis,
        };
    }
};

} // namespace tinygizmo
