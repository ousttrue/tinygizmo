#pragma once
#include <array>

namespace camera
{

struct CameraState
{
    // projection
    float fovYRadians = 60.0f / 180.0f * 3.14f;
    std::array<float, 16> projection;

    // view
    int viewportX = 0;
    int viewportY = 0;
    int viewportWidth = 1;
    int viewportHeight = 1;
    std::array<float, 16> view;

    // viewInverse;
    std::array<float, 3> position;
    std::array<float, 4> rotation;

    // ray for mousecursor
    std::array<float, 3> ray_origin;
    std::array<float, 3> ray_direction;
};

} // namespace camera