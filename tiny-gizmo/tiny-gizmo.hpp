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
    int mouse_x = 0;
    int mouse_y = 0;
    int window_width = 0;
    int window_height = 0;

    bool mouse_left{false};
    bool has_clicked{false};  // State to describe if the user has pressed the left mouse button during the last frame
    bool has_released{false}; // State to describe if the user has released the left mouse button during the last frame

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
    // Used for constructing inverse view projection for raycasting onto gizmo geometry
    camera_parameters cam;
};

///////////////
//   Gizmo   //
///////////////

struct gizmo_context
{
    struct gizmo_context_impl;
    std::unique_ptr<gizmo_context_impl> impl;

    gizmo_context();
    ~gizmo_context();

    // Clear geometry buffer and update internal `gizmo_application_state` data
    void new_frame(const gizmo_application_state &state,
                   const std::array<float, 16> &view, const std::array<float, 16> &projection);
    void render(
        void **pVertices, uint32_t *veticesBytes, uint32_t *vertexStride,
        void **pIndices, uint32_t *indicesBytes, uint32_t *indexStride);

    bool position_gizmo(const std::string &name, struct rigid_transform &t, bool is_local);
    bool orientation_gizmo(const std::string &name, rigid_transform &t, bool is_local);
    bool scale_gizmo(const std::string &name, rigid_transform &t);
};

} // namespace tinygizmo
