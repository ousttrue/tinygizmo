#pragma once
#include "utilmath.h"

namespace tinygizmo
{
///////////////////////
//   Utility Math    //
///////////////////////

static const minalg::float4x4 Identity4x4 = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
static const minalg::float3x3 Identity3x3 = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

void flush_to_zero(minalg::float3 &f)
{
    if (std::abs(f.x) < 0.02f)
        f.x = 0.f;
    if (std::abs(f.y) < 0.02f)
        f.y = 0.f;
    if (std::abs(f.z) < 0.02f)
        f.z = 0.f;
}

// 32 bit Fowler鋒oll坊o Hash
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

minalg::float3 snap(const minalg::float3 &value, const float snap)
{
    if (snap > 0.0f)
        return minalg::float3(floor(value / snap) * snap);
    return value;
}

minalg::float4 make_rotation_quat_axis_angle(const minalg::float3 &axis, float angle)
{
    return {axis * std::sin(angle / 2), std::cos(angle / 2)};
}

minalg::float4 make_rotation_quat_between_vectors_snapped(const minalg::float3 &from, const minalg::float3 &to, const float angle)
{
    auto a = normalize(from);
    auto b = normalize(to);
    auto snappedAcos = std::floor(std::acos(dot(a, b)) / angle) * angle;
    return make_rotation_quat_axis_angle(normalize(cross(a, b)), snappedAcos);
}

ray transform(const rigid_transform &p, const ray &r) { return {p.transform_point(r.origin), p.transform_vector(r.direction)}; }
ray detransform(const rigid_transform &p, const ray &r) { return {p.detransform_point(r.origin), p.detransform_vector(r.direction)}; }

minalg::float3 transform_coord(const minalg::float4x4 &transform, const minalg::float3 &coord)
{
    auto r = mul(transform, minalg::float4(coord, 1));
    return (r.xyz() / r.w);
}
minalg::float3 transform_vector(const minalg::float4x4 &transform, const minalg::float3 &vector) { return mul(transform, minalg::float4(vector, 0)).xyz(); }
void transform(const float scale, ray &r)
{
    r.origin *= scale;
    r.direction *= scale;
}
void detransform(const float scale, ray &r)
{
    r.origin /= scale;
    r.direction /= scale;
}

/////////////////////////////////////////
// Ray-Geometry Intersection Functions //
/////////////////////////////////////////

bool intersect_ray_plane(const ray &ray, const minalg::float4 &plane, float *hit_t)
{
    float denom = dot(plane.xyz(), ray.direction);
    if (std::abs(denom) == 0)
        return false;
    if (hit_t)
        *hit_t = -dot(plane, minalg::float4(ray.origin, 1)) / denom;
    return true;
}

float intersect_ray_triangle(const ray &ray, const minalg::float3 &v0, const minalg::float3 &v1, const minalg::float3 &v2)
{
    auto e1 = v1 - v0, e2 = v2 - v0, h = cross(ray.direction, e2);
    auto a = dot(e1, h);
    if (std::abs(a) == 0)
        return std::numeric_limits<float>::infinity();

    float f = 1 / a;
    auto s = ray.origin - v0;
    auto u = f * dot(s, h);
    if (u < 0 || u > 1)
        return std::numeric_limits<float>::infinity();

    auto q = cross(s, e1);
    auto v = f * dot(ray.direction, q);
    if (v < 0 || u + v > 1)
        return std::numeric_limits<float>::infinity();

    auto t = f * dot(e2, q);
    if (t < 0)
        return std::numeric_limits<float>::infinity();

    return t;
}

float intersect_ray_mesh(const ray &ray, const geometry_mesh &mesh)
{
    float best_t = std::numeric_limits<float>::infinity();
    for (auto &tri : mesh.triangles)
    {
        auto t = intersect_ray_triangle(ray, mesh.vertices[tri[0]].position, mesh.vertices[tri[1]].position, mesh.vertices[tri[2]].position);
        if (t < best_t)
        {
            best_t = t;
        }
    }
    return best_t;
}

} // namespace tinygizmo