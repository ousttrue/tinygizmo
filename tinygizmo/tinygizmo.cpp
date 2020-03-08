// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org>

#include "tinygizmo.h"
#include "geometry_mesh.h"

#include <assert.h>
#include <memory>
#include <vector>
#include <iostream>
#include <functional>
#include <map>
#include <string>
#include <chrono>
#include "impl.h"

namespace tinygizmo
{

gizmo_system::gizmo_system()
    : m_impl(new gizmo_system_impl)
{
}

gizmo_system::~gizmo_system()
{
    delete m_impl;
}

void gizmo_system::new_frame(const gizmo_application_state &state)
{
    m_impl->update(state);
}

void gizmo_system::render(
    void **pVertices, uint32_t *veticesBytes, uint32_t *vertexStride,
    void **pIndices, uint32_t *indicesBytes, uint32_t *indexStride)
{
    auto &r = m_impl->render();
    *pVertices = (void *)r.vertices.data();
    *veticesBytes = static_cast<uint32_t>(r.vertices.size() * sizeof(r.vertices[0]));
    *vertexStride = sizeof(r.vertices[0]);
    *pIndices = (void *)r.triangles.data();
    *indicesBytes = static_cast<uint32_t>(r.triangles.size() * sizeof(r.triangles[0]));
    *indexStride = sizeof(r.triangles[0]);
}

// 32 bit FNV Hash
uint32_t hash_fnv1a(const std::string &str)
{
    static const uint32_t fnv1aBase32 = 0x811C9DC5u;
    static const uint32_t fnv1aPrime32 = 0x01000193u;

    uint32_t result = fnv1aBase32;

    for (auto &c : str)
    {
        result ^= static_cast<uint32_t>(c);
        result *= fnv1aPrime32;
    }
    return result;
}

} // namespace tinygizmo
