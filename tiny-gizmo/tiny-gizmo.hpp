// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org>

#pragma once

#ifndef tinygizmo_hpp
#define tinygizmo_hpp
#include <vector>       // For ... 
#include "minalg.h"

namespace tinygizmo
{

    ///////////////////////
    //   Utility Math    //
    ///////////////////////

    struct rigid_transform
    {
        rigid_transform() {}
        rigid_transform(const minalg::float4 & orientation, const minalg::float3 & position, const minalg::float3 & scale) : orientation(orientation), position(position), scale(scale) {}
        rigid_transform(const minalg::float4 & orientation, const minalg::float3 & position, float scale) : orientation(orientation), position(position), scale(scale) {}
        rigid_transform(const minalg::float4 & orientation, const minalg::float3 & position) : orientation(orientation), position(position) {}

        minalg::float3      position{ 0,0,0 };
        minalg::float4      orientation{ 0,0,0,1 };
        minalg::float3      scale{ 1,1,1 };

        bool                uniform_scale() const { return scale.x == scale.y && scale.x == scale.z; }
        minalg::float4x4    matrix() const { return{ { qxdir(orientation)*scale.x, 0 },{ qydir(orientation)*scale.y, 0 },{ qzdir(orientation)*scale.z,0 },{ position, 1 } }; }
        minalg::float3      transform_vector(const minalg::float3 & vec) const { return qrot(orientation, vec * scale); }
        minalg::float3      transform_point(const minalg::float3 & p) const { return position + transform_vector(p); }
        minalg::float3      detransform_point(const minalg::float3 & p) const { return detransform_vector(p - position); }
        minalg::float3      detransform_vector(const minalg::float3 & vec) const { return qrot(qinv(orientation), vec) / scale; }
    };

    static const float EPSILON = 0.001f;
    inline bool fuzzy_equality(float a, float b, float eps = EPSILON) { return std::abs(a - b) < eps; }
    inline bool fuzzy_equality(minalg::float3 a, minalg::float3 b, float eps = EPSILON) { return fuzzy_equality(a.x, b.x) && fuzzy_equality(a.y, b.y) && fuzzy_equality(a.z, b.z); }
    inline bool fuzzy_equality(minalg::float4 a, minalg::float4 b, float eps = EPSILON) { return fuzzy_equality(a.x, b.x) && fuzzy_equality(a.y, b.y) && fuzzy_equality(a.z, b.z) && fuzzy_equality(a.w, b.w); }
    inline bool operator != (const rigid_transform & a, const rigid_transform & b)
    { 
        return (!fuzzy_equality(a.position, b.position) || !fuzzy_equality(a.orientation, b.orientation) || !fuzzy_equality(a.scale, b.scale));
    }

    struct camera_parameters
    {
        float yfov, near_clip, far_clip;
        minalg::float3 position;
        minalg::float4 orientation;
    };

    struct geometry_vertex { minalg::float3 position, normal; minalg::float4 color; };
    struct geometry_mesh { std::vector<geometry_vertex> vertices; std::vector<minalg::uint3> triangles; };

    ///////////////
    //   Gizmo   //
    ///////////////

    enum class transform_mode
    {
        translate,
        rotate,
        scale
    };

    struct gizmo_application_state
    {
        bool mouse_left{ false };
        bool hotkey_translate{ false };
        bool hotkey_rotate{ false };
        bool hotkey_scale{ false };
        bool hotkey_local{ false };
        bool hotkey_ctrl{ false };
        float screenspace_scale{ 0.f };     // If > 0.f, the gizmos are drawn scale-invariant with a screenspace value defined here
        float snap_translation{ 0.f };      // World-scale units used for snapping translation
        float snap_scale{ 0.f };            // World-scale units used for snapping scale
        float snap_rotation{ 0.f };         // Radians used for snapping rotation quaternions (i.e. PI/8 or PI/16)
        minalg::float2 viewport_size;       // 3d viewport used to render the view
        minalg::float3 ray_origin;          // world-space ray origin (i.e. the camera position)
        minalg::float3 ray_direction;       // world-space ray direction
        camera_parameters cam;              // Used for constructing inverse view projection for raycasting onto gizmo geometry
    };

    struct gizmo_context
    {
        struct gizmo_context_impl;
        std::unique_ptr<gizmo_context_impl> impl;

        gizmo_context();
        ~gizmo_context();

        void update(const gizmo_application_state & state);         // Clear geometry buffer and update internal `gizmo_application_state` data
        const geometry_mesh &render();                                                // Trigger a render callback per call to `update(...)`
        transform_mode get_mode() const;                            // Return the active mode being used by `transform_gizmo(...)`
        // std::function<void(const geometry_mesh & r)> on_render;        // Callback to render the gizmo meshes
    };

    bool transform_gizmo(const std::string & name, gizmo_context & g, rigid_transform & t);

} // end namespace tinygizmo;

#endif // end tinygizmo_hpp
