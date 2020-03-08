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
    minalg::float3 offset;
    rigid_transform original;
    minalg::float3 axis;
};

struct GizmoComponent
{
    geometry_mesh mesh;
    fpalg::float4 base_color;
    fpalg::float4 highlight_color;
    minalg::float3 axis;
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

    void begin(const GizmoComponent *pMesh, const minalg::float3 &offset, const rigid_transform &t, const minalg::float3 &axis)
    {
        m_active = pMesh;
        m_state = {
            .offset = offset,
            .original = t,
            .axis = axis,
        };
    }
};

} // namespace tinygizmo
