#pragma once
#include "minalg.h"
#include "geometry_mesh.h"

namespace tinygizmo
{
///////////////////////
//   Utility Math    //
///////////////////////

// 32 bit Fowler鋒oll坊o Hash
minalg::float3 snap(const minalg::float3 &value, const float snap);
minalg::float4 make_rotation_quat_axis_angle(const minalg::float3 &axis, float angle);
minalg::float4 make_rotation_quat_between_vectors_snapped(const minalg::float3 &from, const minalg::float3 &to, const float angle);

template <typename T>
T clamp(const T &val, const T &min, const T &max) { return std::min(std::max(val, min), max); }

minalg::float3 transform_coord(const minalg::float4x4 &transform, const minalg::float3 &coord);
minalg::float3 transform_vector(const minalg::float4x4 &transform, const minalg::float3 &vector);

} // namespace tinygizmo
