#pragma once
#include "minalg.h"

namespace tinygizmo
{

struct rigid_transform
{
    rigid_transform() {}
    rigid_transform(const minalg::float4 &orientation, const minalg::float3 &position, const minalg::float3 &scale) : orientation(orientation), position(position), scale(scale) {}
    rigid_transform(const minalg::float4 &orientation, const minalg::float3 &position, float scale) : orientation(orientation), position(position), scale(scale) {}
    rigid_transform(const minalg::float4 &orientation, const minalg::float3 &position) : orientation(orientation), position(position) {}

    minalg::float3 position{0, 0, 0};
    minalg::float4 orientation{0, 0, 0, 1};
    minalg::float3 scale{1, 1, 1};

    bool uniform_scale() const { return scale.x == scale.y && scale.x == scale.z; }
    std::array<float, 16> matrix() const
    {
        auto x = qxdir(orientation) * scale.x;
        auto y = qydir(orientation) * scale.y;
        auto z = qzdir(orientation) * scale.z;
        return std::array<float, 16>{
            x.x, x.y, x.z, 0,                      //
            y.x, y.y, y.z, 0,                      //
            z.x, z.y, z.z, 0,                      //
            position.x, position.y, position.z, 1, //
        };
    }
    minalg::float3 transform_vector(const minalg::float3 &vec) const { return qrot(orientation, vec * scale); }
    minalg::float3 transform_point(const minalg::float3 &p) const { return position + transform_vector(p); }
    minalg::float3 detransform_point(const minalg::float3 &p) const { return detransform_vector(p - position); }
    minalg::float3 detransform_vector(const minalg::float3 &vec) const { return qrot(qinv(orientation), vec) / scale; }
};
static const float EPSILON = 0.001f;
inline bool fuzzy_equality(float a, float b, float eps = EPSILON) { return std::abs(a - b) < eps; }
inline bool fuzzy_equality(minalg::float3 a, minalg::float3 b, float eps = EPSILON) { return fuzzy_equality(a.x, b.x) && fuzzy_equality(a.y, b.y) && fuzzy_equality(a.z, b.z); }
inline bool fuzzy_equality(minalg::float4 a, minalg::float4 b, float eps = EPSILON) { return fuzzy_equality(a.x, b.x) && fuzzy_equality(a.y, b.y) && fuzzy_equality(a.z, b.z) && fuzzy_equality(a.w, b.w); }
inline bool operator!=(const rigid_transform &a, const rigid_transform &b)
{
    return (!fuzzy_equality(a.position, b.position) || !fuzzy_equality(a.orientation, b.orientation) || !fuzzy_equality(a.scale, b.scale));
}

} // namespace tinygizmo
