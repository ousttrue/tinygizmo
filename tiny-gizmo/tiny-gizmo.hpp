// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org>

#pragma once
#include <memory>
#include <string>
#include <array>

namespace tinygizmo
{

///////////////
//   State   //
///////////////

struct camera_parameters
{
    // projection
    float yfov;
    float near_clip;
    float far_clip;

    // view
    std::array<float, 3> position;
    std::array<float, 4> orientation;
};

struct gizmo_application_state
{
    bool mouse_left{false};
    bool hotkey_translate{false};
    bool hotkey_rotate{false};
    bool hotkey_scale{false};
    bool hotkey_local{false};
    bool hotkey_ctrl{false};
    // If > 0.f, the gizmos are drawn scale-invariant with a screenspace value defined here
    float screenspace_scale{0.f};
    // World-scale units used for snapping translation
    float snap_translation{0.f};
    // World-scale units used for snapping scale
    float snap_scale{0.f};
    // Radians used for snapping rotation quaternions (i.e. PI/8 or PI/16)
    float snap_rotation{0.f};
    // 3d viewport used to render the view
    std::array<int32_t, 2> viewport_size;
    // world-space ray origin (i.e. the camera position)
    std::array<float, 3> ray_origin;
    // world-space ray direction
    std::array<float, 3> ray_direction;
    // Used for constructing inverse view projection for raycasting onto gizmo geometry
    camera_parameters cam;
};

///////////////
//   Gizmo   //
///////////////

enum class transform_mode
{
    translate,
    rotate,
    scale
};

struct gizmo_context
{
    struct gizmo_context_impl;
    std::unique_ptr<gizmo_context_impl> impl;

    gizmo_context();
    ~gizmo_context();

    void new_frame(const gizmo_application_state &state); // Clear geometry buffer and update internal `gizmo_application_state` data
    void render(
        void **pVertices, uint32_t *veticesBytes, uint32_t *vertexStride,
        void **pIndices, uint32_t *indicesBytes, uint32_t *indexStride);
    transform_mode get_mode() const; // Return the active mode being used by `transform_gizmo(...)`
    bool gizmo(const std::string &name, struct rigid_transform &t);
};

} // namespace tinygizmo
