#pragma once
#include <vector>
#include "minalg.h"
#include <fpalg.h>

namespace tinygizmo
{

struct geometry_vertex
{
    minalg::float3 position;
    minalg::float3 normal;
    minalg::float4 color;
};

struct geometry_mesh
{
    std::vector<geometry_vertex> vertices;
    std::vector<minalg::uint3> triangles;

    static geometry_mesh make_box_geometry(const minalg::float3 &min_bounds, const minalg::float3 &max_bounds);
    static geometry_mesh make_cylinder_geometry(const minalg::float3 &axis, const minalg::float3 &arm1, const minalg::float3 &arm2, uint32_t slices);
    static geometry_mesh make_lathed_geometry(const minalg::float3 &axis, const minalg::float3 &arm1, const minalg::float3 &arm2, int slices, const minalg::float2 *points, uint32_t pointCount, const float eps = 0.0f);

    void compute_normals();

    void clear()
    {
        vertices.clear();
        triangles.clear();
    }
};

float operator>>(const fpalg::Ray &ray, const geometry_mesh &mesh);

} // namespace tinygizmo
