#pragma once
#include <tiny-gizmo.hpp>
#include <castalg.h>
#include <linalg.h>

struct GuiCamera
{
    tinygizmo::camera_parameters params;
    float pitch;
    float yaw;
    std::array<float, 4> get_orientation() const;
    std::array<float, 16> get_viewproj_matrix(const float aspectRatio) const;
    // Returns a world-space ray through the given pixel, originating at the camera
    std::array<float, 3> get_ray_direction(int _x, int _y, int w, int h);
};
