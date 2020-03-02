// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org>

#include "tinygizmo.h"
#include "geometry_mesh.h"
#include "rigid_transform.h"

#include <assert.h>
#include <memory>
#include <vector>
#include <iostream>
#include <functional>
#include <map>
#include <string>
#include <chrono>
#include <castalg.h>
#include "utilmath.h"
#include "impl.h"

namespace tinygizmo
{

gizmo_context::gizmo_context()
    : m_impl(new gizmo_context_impl)
{
}

gizmo_context::~gizmo_context()
{
    delete m_impl;
}

void gizmo_context::new_frame(const gizmo_application_state &state,
                              const std::array<float, 16> &view, const std::array<float, 16> &projection)
{
    m_impl->update(state, view, projection);
}

void gizmo_context::render(
    void **pVertices, uint32_t *veticesBytes, uint32_t *vertexStride,
    void **pIndices, uint32_t *indicesBytes, uint32_t *indexStride)
{
    auto &r = m_impl->render();
    *pVertices = (void *)r.vertices.data();
    *veticesBytes = static_cast<uint32_t>(r.vertices.size() * sizeof(r.vertices[0]));
    *vertexStride = sizeof(r.vertices[0]);
    *pIndices = (void *)r.triangles.data();
    *indicesBytes = static_cast<uint32_t>(r.triangles.size() * sizeof(r.triangles[0]));
    *indexStride = sizeof(r.triangles[0]) / 3;
}

} // namespace tinygizmo
