#pragma once
#include <vector>
#include <fpalg.h>

namespace tinygizmo
{

struct geometry_vertex
{
    fpalg::float3 position;
    fpalg::float3 normal;
    fpalg::float4 color;
};

struct geometry_mesh
{
    std::vector<geometry_vertex> vertices;
    std::vector<uint32_t> triangles;

    static geometry_mesh make_box_geometry(const fpalg::float3 &min_bounds, const fpalg::float3 &max_bounds);
    static geometry_mesh make_cylinder_geometry(const fpalg::float3 &axis, const fpalg::float3 &arm1, const fpalg::float3 &arm2, uint32_t slices);
    static geometry_mesh make_lathed_geometry(const fpalg::float3 &axis, const fpalg::float3 &arm1, const fpalg::float3 &arm2, int slices, const fpalg::float2 *points, uint32_t pointCount, const float eps = 0.0f);

    void compute_normals();

    void clear()
    {
        vertices.clear();
        triangles.clear();
    }
};

float operator>>(const fpalg::Ray &ray, const geometry_mesh &mesh);

} // namespace tinygizmo
