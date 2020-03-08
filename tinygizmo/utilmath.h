#pragma once
#include "minalg.h"

namespace tinygizmo
{

minalg::float3 snap(const minalg::float3 &value, const float snap);
minalg::float4 make_rotation_quat_axis_angle(const minalg::float3 &axis, float angle);
minalg::float4 make_rotation_quat_between_vectors_snapped(const minalg::float3 &from, const minalg::float3 &to, const float angle);

} // namespace tinygizmo
