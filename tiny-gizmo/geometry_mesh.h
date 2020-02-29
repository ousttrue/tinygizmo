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

    static geometry_mesh make_box_geometry(const minalg::float3 &min_bounds, const minalg::float3 &max_bounds);
    static geometry_mesh make_cylinder_geometry(const minalg::float3 &axis, const minalg::float3 &arm1, const minalg::float3 &arm2, uint32_t slices);
    static geometry_mesh make_lathed_geometry(const minalg::float3 &axis, const minalg::float3 &arm1, const minalg::float3 &arm2, int slices, const std::vector<minalg::float2> &points, const float eps = 0.0f);

    void compute_normals();

    void clear()
    {
        vertices.clear();
        triangles.clear();
    }
};

} // namespace tinygizmo
