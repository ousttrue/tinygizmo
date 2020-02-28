#pragma once
#include <vector>
#include "minalg.h"

namespace tinygizmo
{

struct geometry_vertex
{
    minalg::float3 position, normal;
    minalg::float4 color;
};
struct geometry_mesh
{
    std::vector<geometry_vertex> vertices;
    std::vector<minalg::uint3> triangles;
};

} // namespace tinygizmo
