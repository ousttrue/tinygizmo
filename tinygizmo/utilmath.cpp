#pragma once
#include "utilmath.h"

namespace tinygizmo
{

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

} // namespace tinygizmo