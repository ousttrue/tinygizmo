// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org>

#include "util.hpp"
#include "gl-api.hpp"
#include "teapot.h"
#include "window.h"
#include <rigid_transform.h>
#include <castalg.h>

const linalg::aliases::float4x4 identity4x4 = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

constexpr const char gizmo_vert[] = R"(#version 330
    layout(location = 0) in vec3 vertex;
    layout(location = 1) in vec3 normal;
    layout(location = 2) in vec4 color;
    out vec4 v_color;
    out vec3 v_world, v_normal;

    //uniform mat4 u_mvp;
    uniform mat4 u_modelMatrix;
    uniform mat4 u_viewProj;

    void main()
    {
        gl_Position = u_viewProj * u_modelMatrix * vec4(vertex.xyz, 1);
        v_color = color;
        v_world = vertex;
        v_normal = normal;
    }
)";

constexpr const char gizmo_frag[] = R"(#version 330
    in vec4 v_color;
    in vec3 v_world, v_normal;
    out vec4 f_color;
    uniform vec3 u_eye;
    void main()
    {
        vec3 light = vec3(1) * max(dot(v_normal, normalize(u_eye - v_world)), 0.50) + 0.25;
        f_color = v_color * vec4(light, 1);
    }
)";

constexpr const char lit_vert[] = R"(#version 330
    uniform mat4 u_modelMatrix;
    uniform mat4 u_viewProj;

    layout(location = 0) in vec3 inPosition;
    layout(location = 1) in vec3 inNormal;

    out vec3 v_position, v_normal;

    void main()
    {
        vec4 worldPos = u_modelMatrix * vec4(inPosition, 1);
        v_position = worldPos.xyz;
        v_normal = normalize((u_modelMatrix * vec4(inNormal,0)).xyz);
        gl_Position = u_viewProj * worldPos;
    }
)";

constexpr const char lit_frag[] = R"(#version 330
    uniform vec3 u_diffuse = vec3(1, 1, 1);
    uniform vec3 u_eye;

    in vec3 v_position;
    in vec3 v_normal;

    out vec4 f_color;
    
    vec3 compute_lighting(vec3 eyeDir, vec3 position, vec3 color)
    {
        vec3 light = vec3(0, 0, 0);
        vec3 lightDir = normalize(position - v_position);
        light += color * u_diffuse * max(dot(v_normal, lightDir), 0);
        vec3 halfDir = normalize(lightDir + eyeDir);
        light += color * u_diffuse * pow(max(dot(v_normal, halfDir), 0), 128);
        return light;
    }

    void main()
    {
        vec3 eyeDir = vec3(0, 1, -2);
        vec3 light = vec3(0, 0, 0);
        light += compute_lighting(eyeDir, vec3(+3, 1, 0), vec3(235.0/255.0, 43.0/255.0, 211.0/255.0));
        light += compute_lighting(eyeDir, vec3(-3, 1, 0), vec3(43.0/255.0, 236.0/255.0, 234.0/255.0));
        f_color = vec4(light + vec3(0.5, 0.5, 0.5), 1.0);
    }
)";

struct vertex
{
    std::array<float, 3> position;
    std::array<float, 3> normal;
    std::array<float, 4> color;
};
static_assert(sizeof(vertex) == 40, "vertex size");

//////////////////////////
//   Main Application   //
//////////////////////////
int main(int argc, char *argv[])
{
    Window win;
    win.initialize(1280, 800, "tiny-gizmo-example-app");

    GlRenderer renderer;
    if (!renderer.initialize())
    {
        return 1;
    }

    GlModel teapot;
    teapot.shader = std::make_shared<GlShader>(lit_vert, lit_frag);
    std::vector<vertex> vertices;
    for (int i = 0; i < 4974; i += 6)
    {
        vertex v{
            .position{
                teapot_vertices[i + 0],
                teapot_vertices[i + 1],
                teapot_vertices[i + 2]},
            .normal{
                teapot_vertices[i + 3],
                teapot_vertices[i + 4],
                teapot_vertices[i + 5]}};
        vertices.push_back(v);
    }
    teapot.upload_mesh(
        vertices.data(), static_cast<uint32_t>(vertices.size() * sizeof(vertex)), sizeof(vertex),
        teapot_triangles, sizeof(teapot_triangles), sizeof(teapot_triangles[0]),
        false);

    tinygizmo::gizmo_context gizmo_ctx;
    GlModel gizmo;
    gizmo.shader = std::make_shared<GlShader>(gizmo_vert, gizmo_frag);

    camera cam{
        .yfov = 1.0f,
        .near_clip = 0.01f,
        .far_clip = 32.0f,
        .position = {0, 1.5f, 4},
    };

    // teapot a
    tinygizmo::rigid_transform xform_a;
    xform_a.position = {-2, 0, 0};

    // teapot b
    tinygizmo::rigid_transform xform_b;
    xform_b.position = {+2, 0, 0};

    //
    // main loop
    //
    WindowState state;
    WindowState lastState;
    for (int i = 0; win.loop(&state); ++i)
    {
        if (i == 0)
        {
            // skip
        }
        else if (state.mouseRightDown)
        {
            const auto orientation = cam.get_orientation();
            linalg::aliases::float3 move;
            if (state.keycode['W'])
                move -= qzdir(orientation);
            if (state.keycode['A'])
                move -= qxdir(orientation);
            if (state.keycode['S'])
                move += qzdir(orientation);
            if (state.keycode['D'])
                move += qxdir(orientation);
            float timestep = std::chrono::duration<float>(state.time - lastState.time).count();

            if (length2(move) > 0)
                cam.position += normalize(move) * (timestep * 10);

            cam.yaw -= (state.mouseX - lastState.mouseX) * 0.01f;
            cam.pitch -= (state.mouseY - lastState.mouseY) * 0.01f;
        }
        lastState = state;

        const auto cameraOrientation = cam.get_orientation();
        const auto rayDir = get_ray_from_pixel({(float)state.mouseX, (float)state.mouseY}, {0, 0, state.windowWidth, state.windowHeight}, cam).direction;
        tinygizmo::gizmo_application_state gizmo_state{
            .mouse_left = state.mouseLeftDown,
            .hotkey_translate = state.keycode['T'],
            .hotkey_rotate = state.keycode['R'],
            .hotkey_scale = state.keycode['S'],
            .hotkey_local = state.keycode['L'],
            .hotkey_ctrl = state.key_left_control,
            .viewport_size = minalg::float2((float)state.windowWidth, (float)state.windowHeight),
            .ray_origin = minalg::float3(cam.position.x, cam.position.y, cam.position.z),
            .ray_direction = minalg::float3(rayDir.x, rayDir.y, rayDir.z),
            .cam = {
                .yfov = cam.yfov,
                .near_clip = cam.near_clip,
                .far_clip = cam.far_clip,
                .position = castalg::ref_cast<std::array<float, 3>>(cam.position),
                .orientation = castalg::ref_cast<std::array<float, 4>>(cameraOrientation),
            }};
        gizmo_ctx.update(gizmo_state);

        auto viewProjMatrix = cam.get_viewproj_matrix(state.aspectRatio());

        //
        // draw
        //
        renderer.beginFrame(state.windowWidth, state.windowHeight);

        // teapot a
        auto teapotModelMatrix_a_tmp = xform_a.matrix();
        auto teapotModelMatrix_a = reinterpret_cast<const linalg::aliases::float4x4 &>(teapotModelMatrix_a_tmp);
        teapot.draw(cam.position, viewProjMatrix, teapotModelMatrix_a);

        // teapot a
        auto teapotModelMatrix_b_tmp = xform_b.matrix();
        auto teapotModelMatrix_b = reinterpret_cast<const linalg::aliases::float4x4 &>(teapotModelMatrix_b_tmp);
        teapot.draw(cam.position, viewProjMatrix, teapotModelMatrix_b);

        {
            //
            // after scene, before gizmo draw
            // manipulate and update gizmo
            //
            gizmo_ctx.gizmo("first-example-gizmo", xform_a);
            gizmo_ctx.gizmo("second-example-gizmo", xform_b);

            void *pVertices;
            uint32_t verticesBytes;
            uint32_t vertexStride;
            void *pIndices;
            uint32_t indicesBytes;
            uint32_t indexStride;
            gizmo_ctx.render(
                &pVertices, &verticesBytes, &vertexStride,
                &pIndices, &indicesBytes, &indexStride);

            gizmo.upload_mesh(
                pVertices, verticesBytes, vertexStride,
                pIndices, indicesBytes, indexStride,
                true);
        }

        //
        // gizmo after xform user draw
        //
        gizmo.draw(cam.position, viewProjMatrix, identity4x4, true);

        //
        // present
        //
        renderer.endFrame();
        win.swap_buffers();
    }
    return EXIT_SUCCESS;
}
