#pragma once
#include "fpalg.h"

namespace camera
{

struct CameraState
{
    void *userData = nullptr;
    uint32_t UserDataAsUInt() const
    {
        return (uint32_t)(uint64_t)userData;
    }

    std::array<float, 4> clearColor = {0, 0.2f, 0.4f, 1.0f};
    float clearDepth = 1.0f;

    // projection
    float fovYRadians = 60.0f / 180.0f * 3.14f;
    std::array<float, 16> projection;

    // view
    int viewportX =0;
    int viewportY =0;
    int viewportWidth = 1;
    int viewportHeight = 1;
    std::array<float, 16> view;
    // viewInverse;
    std::array<float, 3> position;
    std::array<float, 4> rotation;
    // ray
    std::array<float, 3> ray_origin;
    std::array<float, 3> ray_direction;

    // mult
    std::array<float, 16> viewProjection;
    void CalcViewProjection()
    {
        using fpalg::operator*;
        viewProjection = view * projection;
    }

    std::array<float, 16> CalcModelViewProjection(const std::array<float, 16> &m) const
    {
        using fpalg::operator*;
        return m * viewProjection;
    }
};

} // namespace camera