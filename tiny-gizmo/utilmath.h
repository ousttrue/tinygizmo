#pragma once
#include "minalg.h"
#include "rigid_transform.h"
#include "geometry_mesh.h"

namespace tinygizmo
{
///////////////////////
//   Utility Math    //
///////////////////////

void flush_to_zero(minalg::float3 &f);
// 32 bit Fowler鋒oll坊o Hash
uint32_t hash_fnv1a(const std::string &str);
minalg::float3 snap(const minalg::float3 &value, const float snap);
minalg::float4 make_rotation_quat_axis_angle(const minalg::float3 &axis, float angle);
minalg::float4 make_rotation_quat_between_vectors_snapped(const minalg::float3 &from, const minalg::float3 &to, const float angle);

template <typename T>
T clamp(const T &val, const T &min, const T &max) { return std::min(std::max(val, min), max); }

struct ray
{
    minalg::float3 origin, direction;
};

ray transform(const rigid_transform &p, const ray &r);
ray detransform(const rigid_transform &p, const ray &r);

minalg::float3 transform_coord(const minalg::float4x4 &transform, const minalg::float3 &coord);
minalg::float3 transform_vector(const minalg::float4x4 &transform, const minalg::float3 &vector);
void transform(const float scale, ray &r);
void detransform(const float scale, ray &r);

/////////////////////////////////////////
// Ray-Geometry Intersection Functions //
/////////////////////////////////////////

bool intersect_ray_plane(const ray &ray, const minalg::float4 &plane, float *hit_t);
bool intersect_ray_triangle(const ray &ray, const minalg::float3 &v0, const minalg::float3 &v1, const minalg::float3 &v2, float *hit_t);
bool intersect_ray_mesh(const ray &ray, const geometry_mesh &mesh, float *hit_t);

///////////////////////////////
// Geometry + Mesh Utilities //
///////////////////////////////
void compute_normals(geometry_mesh &mesh);
geometry_mesh make_box_geometry(const minalg::float3 &min_bounds, const minalg::float3 &max_bounds);
geometry_mesh make_cylinder_geometry(const minalg::float3 &axis, const minalg::float3 &arm1, const minalg::float3 &arm2, uint32_t slices);
geometry_mesh make_lathed_geometry(const minalg::float3 &axis, const minalg::float3 &arm1, const minalg::float3 &arm2, int slices, const std::vector<minalg::float2> &points, const float eps = 0.0f);

} // namespace tinygizmo