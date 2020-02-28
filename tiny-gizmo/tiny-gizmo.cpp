// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org>

#include "tiny-gizmo.hpp"
#include "rigid_transform.h"

#include <assert.h>
#include <memory>
#include <vector>
#include <iostream>
#include <functional>
#include <map>
#include <string>
#include <chrono>
#include <castalg.h>

using namespace tinygizmo;

#include "utilmath.h"

using namespace minalg;

struct gizmo_mesh_component
{
    geometry_mesh mesh;
    float4 base_color, highlight_color;
};
struct gizmo_renderable
{
    geometry_mesh mesh;
    float4 color;
};

//////////////////////////////////
// Gizmo Context Implementation //
//////////////////////////////////

enum class interact
{
    none,
    translate_x,
    translate_y,
    translate_z,
    translate_yz,
    translate_zx,
    translate_xy,
    translate_xyz,
    rotate_x,
    rotate_y,
    rotate_z,
    scale_x,
    scale_y,
    scale_z,
    scale_xyz,
};

struct interaction_state
{
    bool active{false};          // Flag to indicate if the gizmo is being actively manipulated
    bool hover{false};           // Flag to indicate if the gizmo is being hovered
    float3 original_position;    // Original position of an object being manipulated with a gizmo
    float4 original_orientation; // Original orientation of an object being manipulated with a gizmo
    float3 original_scale;       // Original scale of an object being manipulated with a gizmo
    float3 click_offset;         // Offset from position of grabbed object to coordinates of clicked point
    interact interaction_mode;   // Currently active component
};

struct gizmo_context::gizmo_context_impl
{
private:
    gizmo_context *ctx;
    tinygizmo::geometry_mesh m_r{};

public:
    std::map<interact, gizmo_mesh_component> mesh_components;
    std::vector<gizmo_renderable> drawlist;

    transform_mode mode{transform_mode::translate};

    std::map<uint32_t, interaction_state> gizmos;

    gizmo_application_state active_state;
    gizmo_application_state last_state;
    bool local_toggle{true};  // State to describe if the gizmo should use transform-local math
    bool has_clicked{false};  // State to describe if the user has pressed the left mouse button during the last frame
    bool has_released{false}; // State to describe if the user has released the left mouse button during the last frame

    gizmo_context_impl(gizmo_context *ctx) : ctx(ctx)
    {
        std::vector<float2> arrow_points = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.10f}, {1.2f, 0}};
        std::vector<float2> mace_points = {{0.25f, 0}, {0.25f, 0.05f}, {1, 0.05f}, {1, 0.1f}, {1.25f, 0.1f}, {1.25f, 0}};
        std::vector<float2> ring_points = {{+0.025f, 1}, {-0.025f, 1}, {-0.025f, 1}, {-0.025f, 1.1f}, {-0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1.1f}, {+0.025f, 1}};
        mesh_components[interact::translate_x] = {make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, arrow_points), {1, 0.5f, 0.5f, 1.f}, {1, 0, 0, 1.f}};
        mesh_components[interact::translate_y] = {make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, arrow_points), {0.5f, 1, 0.5f, 1.f}, {0, 1, 0, 1.f}};
        mesh_components[interact::translate_z] = {make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, arrow_points), {0.5f, 0.5f, 1, 1.f}, {0, 0, 1, 1.f}};
        mesh_components[interact::translate_yz] = {make_box_geometry({-0.01f, 0.25, 0.25}, {0.01f, 0.75f, 0.75f}), {0.5f, 1, 1, 0.5f}, {0, 1, 1, 0.6f}};
        mesh_components[interact::translate_zx] = {make_box_geometry({0.25, -0.01f, 0.25}, {0.75f, 0.01f, 0.75f}), {1, 0.5f, 1, 0.5f}, {1, 0, 1, 0.6f}};
        mesh_components[interact::translate_xy] = {make_box_geometry({0.25, 0.25, -0.01f}, {0.75f, 0.75f, 0.01f}), {1, 1, 0.5f, 0.5f}, {1, 1, 0, 0.6f}};
        mesh_components[interact::translate_xyz] = {make_box_geometry({-0.05f, -0.05f, -0.05f}, {0.05f, 0.05f, 0.05f}), {0.9f, 0.9f, 0.9f, 0.25f}, {1, 1, 1, 0.35f}};
        mesh_components[interact::rotate_x] = {make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 32, ring_points, 0.003f), {1, 0.5f, 0.5f, 1.f}, {1, 0, 0, 1.f}};
        mesh_components[interact::rotate_y] = {make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 32, ring_points, -0.003f), {0.5f, 1, 0.5f, 1.f}, {0, 1, 0, 1.f}};
        mesh_components[interact::rotate_z] = {make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 32, ring_points), {0.5f, 0.5f, 1, 1.f}, {0, 0, 1, 1.f}};
        mesh_components[interact::scale_x] = {make_lathed_geometry({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 16, mace_points), {1, 0.5f, 0.5f, 1.f}, {1, 0, 0, 1.f}};
        mesh_components[interact::scale_y] = {make_lathed_geometry({0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 16, mace_points), {0.5f, 1, 0.5f, 1.f}, {0, 1, 0, 1.f}};
        mesh_components[interact::scale_z] = {make_lathed_geometry({0, 0, 1}, {1, 0, 0}, {0, 1, 0}, 16, mace_points), {0.5f, 0.5f, 1, 1.f}, {0, 0, 1, 1.f}};
    }

    // Public methods
    void gizmo_context_impl::update(const gizmo_application_state &state)
    {
        active_state = state;
        local_toggle = (!last_state.hotkey_local && active_state.hotkey_local && active_state.hotkey_ctrl) ? !local_toggle : local_toggle;
        has_clicked = (!last_state.mouse_left && active_state.mouse_left) ? true : false;
        has_released = (last_state.mouse_left && !active_state.mouse_left) ? true : false;
        drawlist.clear();
    }

    const geometry_mesh &render()
    {
        m_r.vertices.clear();
        m_r.triangles.clear();

        // geometry_mesh r;
        // Combine all gizmo sub-meshes into one super-mesh
        for (auto &m : drawlist)
        {
            uint32_t numVerts = (uint32_t)m_r.vertices.size();
            auto it = m_r.vertices.insert(m_r.vertices.end(), m.mesh.vertices.begin(), m.mesh.vertices.end());
            for (auto &f : m.mesh.triangles)
                m_r.triangles.push_back({numVerts + f.x, numVerts + f.y, numVerts + f.z});
            for (; it != m_r.vertices.end(); ++it)
                it->color = m.color; // Take the color and shove it into a per-vertex attribute
        }

        last_state = active_state;

        return m_r;
    }
};

// This will calculate a scale constant based on the number of screenspace pixels passed as pixel_scale.
float scale_screenspace(gizmo_context::gizmo_context_impl &g, const float3 position, const float pixel_scale)
{
    float dist = length(position - castalg::ref_cast<minalg::float3>(g.active_state.cam.position));
    return std::tan(g.active_state.cam.yfov) * dist * (pixel_scale / g.active_state.viewport_size.y);
}

// The only purpose of this is readability: to reduce the total column width of the intersect(...) statements in every gizmo
bool intersect(gizmo_context::gizmo_context_impl &g, const ray &r, interact i, float &t, const float best_t)
{
    if (intersect_ray_mesh(r, g.mesh_components[i].mesh, &t) && t < best_t)
        return true;
    return false;
}

///////////////////////////////////
// Private Gizmo Implementations //
///////////////////////////////////

void axis_rotation_dragger(const uint32_t id, gizmo_context::gizmo_context_impl &g, const float3 &axis, const float3 &center, const float4 &start_orientation, float4 &orientation)
{
    interaction_state &interaction = g.gizmos[id];

    if (g.active_state.mouse_left)
    {
        rigid_transform original_pose = {start_orientation, interaction.original_position};
        float3 the_axis = original_pose.transform_vector(axis);
        float4 the_plane = {the_axis, -dot(the_axis, interaction.click_offset)};
        const ray r = {g.active_state.ray_origin, g.active_state.ray_direction};

        float t;
        if (intersect_ray_plane(r, the_plane, &t))
        {
            float3 center_of_rotation = interaction.original_position + the_axis * dot(the_axis, interaction.click_offset - interaction.original_position);
            float3 arm1 = normalize(interaction.click_offset - center_of_rotation);
            float3 arm2 = normalize(r.origin + r.direction * t - center_of_rotation);

            float d = dot(arm1, arm2);
            if (d > 0.999f)
            {
                orientation = start_orientation;
                return;
            }

            float angle = std::acos(d);
            if (angle < 0.001f)
            {
                orientation = start_orientation;
                return;
            }

            if (g.active_state.snap_rotation)
            {
                auto snapped = make_rotation_quat_between_vectors_snapped(arm1, arm2, g.active_state.snap_rotation);
                orientation = qmul(snapped, start_orientation);
            }
            else
            {
                auto a = normalize(cross(arm1, arm2));
                orientation = qmul(rotation_quat(a, angle), start_orientation);
            }
        }
    }
}

void plane_translation_dragger(const uint32_t id, gizmo_context::gizmo_context_impl &g, const float3 &plane_normal, float3 &point)
{
    interaction_state &interaction = g.gizmos[id];

    // Mouse clicked
    if (g.has_clicked)
        interaction.original_position = point;

    if (g.active_state.mouse_left)
    {
        // Define the plane to contain the original position of the object
        const float3 plane_point = interaction.original_position;
        const ray r = {g.active_state.ray_origin, g.active_state.ray_direction};

        // If an intersection exists between the ray and the plane, place the object at that point
        const float denom = dot(r.direction, plane_normal);
        if (std::abs(denom) == 0)
            return;

        const float t = dot(plane_point - r.origin, plane_normal) / denom;
        if (t < 0)
            return;

        point = r.origin + r.direction * t;

        if (g.active_state.snap_translation)
            point = snap(point, g.active_state.snap_translation);
    }
}

void axis_translation_dragger(const uint32_t id, gizmo_context::gizmo_context_impl &g, const float3 &axis, float3 &point)
{
    interaction_state &interaction = g.gizmos[id];

    if (g.active_state.mouse_left)
    {
        // First apply a plane translation dragger with a plane that contains the desired axis and is oriented to face the camera
        const float3 plane_tangent = cross(axis, point - castalg::ref_cast<minalg::float3>(g.active_state.cam.position));
        const float3 plane_normal = cross(axis, plane_tangent);
        plane_translation_dragger(id, g, plane_normal, point);

        // Constrain object motion to be along the desired axis
        point = interaction.original_position + axis * dot(point - interaction.original_position, axis);
    }
}

///////////////////////////////
//   Gizmo Implementations   //
///////////////////////////////

void position_gizmo(const std::string &name, gizmo_context::gizmo_context_impl &g, const float4 &orientation, float3 &position)
{
    rigid_transform p = rigid_transform(g.local_toggle ? orientation : float4(0, 0, 0, 1), position);
    const float draw_scale = (g.active_state.screenspace_scale > 0.f) ? scale_screenspace(g, p.position, g.active_state.screenspace_scale) : 1.f;
    const uint32_t id = hash_fnv1a(name);

    // interaction_mode will only change on clicked
    if (g.has_clicked)
        g.gizmos[id].interaction_mode = interact::none;

    {
        interact updated_state = interact::none;
        auto ray = detransform(p, {g.active_state.ray_origin, g.active_state.ray_direction});
        detransform(draw_scale, ray);

        float best_t = std::numeric_limits<float>::infinity(), t;
        if (intersect(g, ray, interact::translate_x, t, best_t))
        {
            updated_state = interact::translate_x;
            best_t = t;
        }
        if (intersect(g, ray, interact::translate_y, t, best_t))
        {
            updated_state = interact::translate_y;
            best_t = t;
        }
        if (intersect(g, ray, interact::translate_z, t, best_t))
        {
            updated_state = interact::translate_z;
            best_t = t;
        }
        if (intersect(g, ray, interact::translate_yz, t, best_t))
        {
            updated_state = interact::translate_yz;
            best_t = t;
        }
        if (intersect(g, ray, interact::translate_zx, t, best_t))
        {
            updated_state = interact::translate_zx;
            best_t = t;
        }
        if (intersect(g, ray, interact::translate_xy, t, best_t))
        {
            updated_state = interact::translate_xy;
            best_t = t;
        }
        if (intersect(g, ray, interact::translate_xyz, t, best_t))
        {
            updated_state = interact::translate_xyz;
            best_t = t;
        }

        if (g.has_clicked)
        {
            g.gizmos[id].interaction_mode = updated_state;

            if (g.gizmos[id].interaction_mode != interact::none)
            {
                transform(draw_scale, ray);
                g.gizmos[id].click_offset = g.local_toggle ? p.transform_vector(ray.origin + ray.direction * t) : ray.origin + ray.direction * t;
                g.gizmos[id].active = true;
            }
            else
                g.gizmos[id].active = false;
        }

        g.gizmos[id].hover = (best_t == std::numeric_limits<float>::infinity()) ? false : true;
    }

    std::vector<float3> axes;
    if (g.local_toggle)
        axes = {qxdir(p.orientation), qydir(p.orientation), qzdir(p.orientation)};
    else
        axes = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    if (g.gizmos[id].active)
    {
        position += g.gizmos[id].click_offset;
        switch (g.gizmos[id].interaction_mode)
        {
        case interact::translate_x:
            axis_translation_dragger(id, g, axes[0], position);
            break;
        case interact::translate_y:
            axis_translation_dragger(id, g, axes[1], position);
            break;
        case interact::translate_z:
            axis_translation_dragger(id, g, axes[2], position);
            break;
        case interact::translate_yz:
            plane_translation_dragger(id, g, axes[0], position);
            break;
        case interact::translate_zx:
            plane_translation_dragger(id, g, axes[1], position);
            break;
        case interact::translate_xy:
            plane_translation_dragger(id, g, axes[2], position);
            break;
        case interact::translate_xyz:
            plane_translation_dragger(id, g, -minalg::qzdir(castalg::ref_cast<minalg::float4>(g.active_state.cam.orientation)), position);
            break;
        }
        position -= g.gizmos[id].click_offset;
    }

    if (g.has_released)
    {
        g.gizmos[id].interaction_mode = interact::none;
        g.gizmos[id].active = false;
    }

    std::vector<interact> draw_interactions{
        interact::translate_x, interact::translate_y, interact::translate_z,
        interact::translate_yz, interact::translate_zx, interact::translate_xy,
        interact::translate_xyz};

    float4x4 modelMatrix = p.matrix();
    float4x4 scaleMatrix = scaling_matrix(float3(draw_scale));
    modelMatrix = mul(modelMatrix, scaleMatrix);

    for (auto c : draw_interactions)
    {
        gizmo_renderable r;
        r.mesh = g.mesh_components[c].mesh;
        r.color = (c == g.gizmos[id].interaction_mode) ? g.mesh_components[c].base_color : g.mesh_components[c].highlight_color;
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        g.drawlist.push_back(r);
    }
}

void orientation_gizmo(const std::string &name, gizmo_context::gizmo_context_impl &g, const float3 &center, float4 &orientation)
{
    assert(length2(orientation) > float(1e-6));

    rigid_transform p = rigid_transform(g.local_toggle ? orientation : float4(0, 0, 0, 1), center); // Orientation is local by default
    const float draw_scale = (g.active_state.screenspace_scale > 0.f) ? scale_screenspace(g, p.position, g.active_state.screenspace_scale) : 1.f;
    const uint32_t id = hash_fnv1a(name);

    // interaction_mode will only change on clicked
    if (g.has_clicked)
        g.gizmos[id].interaction_mode = interact::none;

    {
        interact updated_state = interact::none;

        auto ray = detransform(p, {g.active_state.ray_origin, g.active_state.ray_direction});
        detransform(draw_scale, ray);
        float best_t = std::numeric_limits<float>::infinity(), t;

        if (intersect(g, ray, interact::rotate_x, t, best_t))
        {
            updated_state = interact::rotate_x;
            best_t = t;
        }
        if (intersect(g, ray, interact::rotate_y, t, best_t))
        {
            updated_state = interact::rotate_y;
            best_t = t;
        }
        if (intersect(g, ray, interact::rotate_z, t, best_t))
        {
            updated_state = interact::rotate_z;
            best_t = t;
        }

        if (g.has_clicked)
        {
            g.gizmos[id].interaction_mode = updated_state;
            if (g.gizmos[id].interaction_mode != interact::none)
            {
                transform(draw_scale, ray);
                g.gizmos[id].original_position = center;
                g.gizmos[id].original_orientation = orientation;
                g.gizmos[id].click_offset = p.transform_point(ray.origin + ray.direction * t);
                g.gizmos[id].active = true;
            }
            else
                g.gizmos[id].active = false;
        }
    }

    float3 activeAxis;
    if (g.gizmos[id].active)
    {
        const float4 starting_orientation = g.local_toggle ? g.gizmos[id].original_orientation : float4(0, 0, 0, 1);
        switch (g.gizmos[id].interaction_mode)
        {
        case interact::rotate_x:
            axis_rotation_dragger(id, g, {1, 0, 0}, center, starting_orientation, p.orientation);
            activeAxis = {1, 0, 0};
            break;
        case interact::rotate_y:
            axis_rotation_dragger(id, g, {0, 1, 0}, center, starting_orientation, p.orientation);
            activeAxis = {0, 1, 0};
            break;
        case interact::rotate_z:
            axis_rotation_dragger(id, g, {0, 0, 1}, center, starting_orientation, p.orientation);
            activeAxis = {0, 0, 1};
            break;
        }
    }

    if (g.has_released)
    {
        g.gizmos[id].interaction_mode = interact::none;
        g.gizmos[id].active = false;
    }

    float4x4 modelMatrix = p.matrix();
    float4x4 scaleMatrix = scaling_matrix(float3(draw_scale));
    modelMatrix = mul(modelMatrix, scaleMatrix);

    std::vector<interact> draw_interactions;
    if (!g.local_toggle && g.gizmos[id].interaction_mode != interact::none)
        draw_interactions = {g.gizmos[id].interaction_mode};
    else
        draw_interactions = {interact::rotate_x, interact::rotate_y, interact::rotate_z};

    for (auto c : draw_interactions)
    {
        gizmo_renderable r;
        r.mesh = g.mesh_components[c].mesh;
        r.color = (c == g.gizmos[id].interaction_mode) ? g.mesh_components[c].base_color : g.mesh_components[c].highlight_color;
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        g.drawlist.push_back(r);
    }

    // For non-local transformations, we only present one rotation ring
    // and draw an arrow from the center of the gizmo to indicate the degree of rotation
    if (g.local_toggle == false && g.gizmos[id].interaction_mode != interact::none)
    {
        interaction_state &interaction = g.gizmos[id];

        // Create orthonormal basis for drawing the arrow
        float3 a = qrot(p.orientation, interaction.click_offset - interaction.original_position);
        float3 zDir = normalize(activeAxis), xDir = normalize(cross(a, zDir)), yDir = cross(zDir, xDir);

        // Ad-hoc geometry
        std::initializer_list<float2> arrow_points = {{0.0f, 0.f}, {0.0f, 0.05f}, {0.8f, 0.05f}, {0.9f, 0.10f}, {1.0f, 0}};
        auto geo = make_lathed_geometry(yDir, xDir, zDir, 32, arrow_points);

        gizmo_renderable r;
        r.mesh = geo;
        r.color = float4(1);
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position);
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        g.drawlist.push_back(r);

        orientation = qmul(p.orientation, interaction.original_orientation);
    }
    else if (g.local_toggle == true && g.gizmos[id].interaction_mode != interact::none)
        orientation = p.orientation;
}

void axis_scale_dragger(const uint32_t &id, gizmo_context::gizmo_context_impl &g, const float3 &axis, const float3 &center, float3 &scale, const bool uniform)
{
    interaction_state &interaction = g.gizmos[id];

    if (g.active_state.mouse_left)
    {
        const float3 plane_tangent = cross(axis, center - castalg::ref_cast<minalg::float3>(g.active_state.cam.position));
        const float3 plane_normal = cross(axis, plane_tangent);

        float3 distance;
        if (g.active_state.mouse_left)
        {
            // Define the plane to contain the original position of the object
            const float3 plane_point = center;
            const ray ray = {g.active_state.ray_origin, g.active_state.ray_direction};

            // If an intersection exists between the ray and the plane, place the object at that point
            const float denom = dot(ray.direction, plane_normal);
            if (std::abs(denom) == 0)
                return;

            const float t = dot(plane_point - ray.origin, plane_normal) / denom;
            if (t < 0)
                return;

            distance = ray.origin + ray.direction * t;
        }

        float3 offset_on_axis = (distance - interaction.click_offset) * axis;
        flush_to_zero(offset_on_axis);
        float3 new_scale = interaction.original_scale + offset_on_axis;

        if (uniform)
            scale = float3(clamp(dot(distance, new_scale), 0.01f, 1000.f));
        else
            scale = float3(clamp(new_scale.x, 0.01f, 1000.f), clamp(new_scale.y, 0.01f, 1000.f), clamp(new_scale.z, 0.01f, 1000.f));
        if (g.active_state.snap_scale)
            scale = snap(scale, g.active_state.snap_scale);
    }
}

void scale_gizmo(const std::string &name, gizmo_context::gizmo_context_impl &g, const float4 &orientation, const float3 &center, float3 &scale)
{
    rigid_transform p = rigid_transform(orientation, center);
    const float draw_scale = (g.active_state.screenspace_scale > 0.f) ? scale_screenspace(g, p.position, g.active_state.screenspace_scale) : 1.f;
    const uint32_t id = hash_fnv1a(name);

    if (g.has_clicked)
        g.gizmos[id].interaction_mode = interact::none;

    {
        interact updated_state = interact::none;
        auto ray = detransform(p, {g.active_state.ray_origin, g.active_state.ray_direction});
        detransform(draw_scale, ray);
        float best_t = std::numeric_limits<float>::infinity(), t;
        if (intersect(g, ray, interact::scale_x, t, best_t))
        {
            updated_state = interact::scale_x;
            best_t = t;
        }
        if (intersect(g, ray, interact::scale_y, t, best_t))
        {
            updated_state = interact::scale_y;
            best_t = t;
        }
        if (intersect(g, ray, interact::scale_z, t, best_t))
        {
            updated_state = interact::scale_z;
            best_t = t;
        }

        if (g.has_clicked)
        {
            g.gizmos[id].interaction_mode = updated_state;
            if (g.gizmos[id].interaction_mode != interact::none)
            {
                transform(draw_scale, ray);
                g.gizmos[id].original_scale = scale;
                g.gizmos[id].click_offset = p.transform_point(ray.origin + ray.direction * t);
                g.gizmos[id].active = true;
            }
            else
                g.gizmos[id].active = false;
        }
    }

    if (g.has_released)
    {
        g.gizmos[id].interaction_mode = interact::none;
        g.gizmos[id].active = false;
    }

    if (g.gizmos[id].active)
    {
        switch (g.gizmos[id].interaction_mode)
        {
        case interact::scale_x:
            axis_scale_dragger(id, g, {1, 0, 0}, center, scale, g.active_state.hotkey_ctrl);
            break;
        case interact::scale_y:
            axis_scale_dragger(id, g, {0, 1, 0}, center, scale, g.active_state.hotkey_ctrl);
            break;
        case interact::scale_z:
            axis_scale_dragger(id, g, {0, 0, 1}, center, scale, g.active_state.hotkey_ctrl);
            break;
        }
    }

    float4x4 modelMatrix = p.matrix();
    float4x4 scaleMatrix = scaling_matrix(float3(draw_scale));
    modelMatrix = mul(modelMatrix, scaleMatrix);

    std::vector<interact> draw_components{interact::scale_x, interact::scale_y, interact::scale_z};

    for (auto c : draw_components)
    {
        gizmo_renderable r;
        r.mesh = g.mesh_components[c].mesh;
        r.color = (c == g.gizmos[id].interaction_mode) ? g.mesh_components[c].base_color : g.mesh_components[c].highlight_color;
        for (auto &v : r.mesh.vertices)
        {
            v.position = transform_coord(modelMatrix, v.position); // transform local coordinates into worldspace
            v.normal = transform_vector(modelMatrix, v.normal);
        }
        g.drawlist.push_back(r);
    }
}

//////////////////////////////////
// Public Gizmo Implementations //
//////////////////////////////////

gizmo_context::gizmo_context() { impl.reset(new gizmo_context_impl(this)); };
gizmo_context::~gizmo_context() {}
void gizmo_context::update(const gizmo_application_state &state) { impl->update(state); }
const tinygizmo::geometry_mesh &gizmo_context::render() { return impl->render(); }
transform_mode gizmo_context::get_mode() const { return impl->mode; }

bool tinygizmo::gizmo_context::gizmo(const std::string &name, rigid_transform &t)
{
    bool activated = false;

    if (this->impl->active_state.hotkey_ctrl == true)
    {
        if (this->impl->last_state.hotkey_translate == false && this->impl->active_state.hotkey_translate == true)
            this->impl->mode = transform_mode::translate;
        else if (this->impl->last_state.hotkey_rotate == false && this->impl->active_state.hotkey_rotate == true)
            this->impl->mode = transform_mode::rotate;
        else if (this->impl->last_state.hotkey_scale == false && this->impl->active_state.hotkey_scale == true)
            this->impl->mode = transform_mode::scale;
    }

    if (this->impl->mode == transform_mode::translate)
        position_gizmo(name, *this->impl, t.orientation, t.position);
    else if (this->impl->mode == transform_mode::rotate)
        orientation_gizmo(name, *this->impl, t.position, t.orientation);
    else if (this->impl->mode == transform_mode::scale)
        scale_gizmo(name, *this->impl, t.orientation, t.position, t.scale);

    const interaction_state s = this->impl->gizmos[hash_fnv1a(name)];
    if (s.hover == true || s.active == true)
        activated = true;

    return activated;
}