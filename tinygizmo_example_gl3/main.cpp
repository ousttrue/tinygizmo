// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org>

#include "renderer.h"
#include "wgl_context.h"
#include "teapot.h"
#include <vector>
#include <tinygizmo.h>
#include <Win32Window.h>
#include <orbit_camera.h>

struct vertex
{
    std::array<float, 3> position;
    std::array<float, 3> normal;
    std::array<float, 4> color;
};
static_assert(sizeof(vertex) == 40, "vertex size");

enum class transform_mode
{
    translate,
    rotate,
    scale
};

static std::array<float, 16> mul(const std::array<float, 16> &lhs, const std::array<float, 16> &rhs)
{
    return {
        lhs[0] * rhs[0] + lhs[1] * rhs[4] + lhs[2] * rhs[8] + lhs[3] * rhs[12],
        lhs[0] * rhs[1] + lhs[1] * rhs[5] + lhs[2] * rhs[9] + lhs[3] * rhs[13],
        lhs[0] * rhs[2] + lhs[1] * rhs[6] + lhs[2] * rhs[10] + lhs[3] * rhs[14],
        lhs[0] * rhs[3] + lhs[1] * rhs[7] + lhs[2] * rhs[11] + lhs[3] * rhs[15],

        lhs[4] * rhs[0] + lhs[5] * rhs[4] + lhs[6] * rhs[8] + lhs[7] * rhs[12],
        lhs[4] * rhs[1] + lhs[5] * rhs[5] + lhs[6] * rhs[9] + lhs[7] * rhs[13],
        lhs[4] * rhs[2] + lhs[5] * rhs[6] + lhs[6] * rhs[10] + lhs[7] * rhs[14],
        lhs[4] * rhs[3] + lhs[5] * rhs[7] + lhs[6] * rhs[11] + lhs[7] * rhs[15],

        lhs[8] * rhs[0] + lhs[9] * rhs[4] + lhs[10] * rhs[8] + lhs[11] * rhs[12],
        lhs[8] * rhs[1] + lhs[9] * rhs[5] + lhs[10] * rhs[9] + lhs[11] * rhs[13],
        lhs[8] * rhs[2] + lhs[9] * rhs[6] + lhs[10] * rhs[10] + lhs[11] * rhs[14],
        lhs[8] * rhs[3] + lhs[9] * rhs[7] + lhs[10] * rhs[11] + lhs[11] * rhs[15],

        lhs[12] * rhs[0] + lhs[13] * rhs[4] + lhs[14] * rhs[8] + lhs[15] * rhs[12],
        lhs[12] * rhs[1] + lhs[13] * rhs[5] + lhs[14] * rhs[9] + lhs[15] * rhs[13],
        lhs[12] * rhs[2] + lhs[13] * rhs[6] + lhs[14] * rhs[10] + lhs[15] * rhs[14],
        lhs[12] * rhs[3] + lhs[13] * rhs[7] + lhs[14] * rhs[11] + lhs[15] * rhs[15],
    };
}


//////////////////////////
//   Main Application   //
//////////////////////////
int main(int argc, char *argv[])
{
    screenstate::Win32Window win(L"tinygizmo_example_class");
    auto hwnd = win.Create(L"tinygizmo_example-app", 1280, 800);
    if (!hwnd)
    {
        return 1;
    }
    win.Show();

    WGLContext wgl;
    if (!wgl.Create(hwnd, 3, 0))
    {
        return 2;
    }    

    // camera
    OrbitCamera camera{};
    transform_mode mode = transform_mode::scale;
    bool is_local = true;

    //
    // scene
    //
    Renderer renderer;
    if (!renderer.initialize())
    {
        return 3;
    }

    // create teapot
    auto teapot_mesh = renderer.createMesh();
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
    teapot_mesh->uploadMesh(
        vertices.data(), static_cast<uint32_t>(vertices.size() * sizeof(vertex)), sizeof(vertex),
        teapot_triangles, sizeof(teapot_triangles), sizeof(teapot_triangles[0]),
        false);

    // teapot a
    fpalg::TRS teapot_a{
        {
            .position{-2, 0, 0}
        }
    };

    // teapot b
    fpalg::TRS teapot_b{
        {
            .position = {+2, 0, 0},
        }
    };

    // gizmo
    tinygizmo::gizmo_system gizmo_system;
    auto gizmo_mesh = renderer.createMeshForGizmo();

    //
    // main loop
    //
    screenstate::ScreenState state;
    screenstate::ScreenState lastState{};
    for (int i = 0; win.Update(&state); ++i)
    {
        // update camera
        camera.WindowInput(state);

        // gizmo new frame
        tinygizmo::gizmo_application_state gizmo_state{
            .window_width = state.Width,
            .window_height = state.Height,
            .mouse_x = state.MouseX,
            .mouse_y = state.MouseY,
            .mouse_left = state.MouseLeftDown(),
            // .hotkey_ctrl = state.key_left_control,
            .camera_position = camera.state.position,
            .camera_orientation = camera.state.rotation,
        };
        gizmo_state.has_clicked = !lastState.MouseLeftDown() && state.MouseLeftDown();
        gizmo_state.has_released = lastState.MouseLeftDown() && !state.MouseLeftDown();

        gizmo_system.new_frame(gizmo_state, camera.state.viewProjection);

        if (state.KeyCode['R'])
        {
            mode = transform_mode::rotate;
        }
        if (state.KeyCode['T'])
        {
            mode = transform_mode::translate;
        }
        if (state.KeyCode['S'])
        {
            mode = transform_mode::scale;
        }
        if (!lastState.KeyCode['Z'] && state.KeyCode['Z'])
        {
            is_local = !is_local;
        }
        lastState = state;

        //
        // draw
        //
        renderer.beginFrame(state.Width, state.Height);
        teapot_mesh->draw(teapot_a.Matrix().data(), camera.state.viewProjection.data(), camera.state.position.data());
        teapot_mesh->draw(teapot_b.Matrix().data(), camera.state.viewProjection.data(), camera.state.position.data());

        {
            //
            // after scene, before gizmo draw
            // manipulate and update gizmo
            //
            switch (mode)
            {
            case transform_mode::translate:
                tinygizmo::position_gizmo(gizmo_system, "first-example-gizmo", teapot_a, is_local);
                tinygizmo::position_gizmo(gizmo_system, "second-example-gizmo", teapot_b, is_local);
                break;

            case transform_mode::rotate:
                tinygizmo::orientation_gizmo(gizmo_system, "first-example-gizmo", teapot_a, is_local);
                tinygizmo::orientation_gizmo(gizmo_system, "second-example-gizmo", teapot_b, is_local);
                break;

            case transform_mode::scale:
                tinygizmo::scale_gizmo(gizmo_system, "first-example-gizmo", teapot_a, is_local);
                tinygizmo::scale_gizmo(gizmo_system, "second-example-gizmo", teapot_b, is_local);
                break;
            }

            void *pVertices;
            uint32_t verticesBytes;
            uint32_t vertexStride;
            void *pIndices;
            uint32_t indicesBytes;
            uint32_t indexStride;
            gizmo_system.render(
                &pVertices, &verticesBytes, &vertexStride,
                &pIndices, &indicesBytes, &indexStride);

            gizmo_mesh->uploadMesh(
                pVertices, verticesBytes, vertexStride,
                pIndices, indicesBytes, indexStride,
                true);
        }

        //
        // gizmo after xform user draw
        //
        renderer.clearDepth();
        static const float identity4x4[] = {
            1, 0, 0, 0, //
            0, 1, 0, 0, //
            0, 0, 1, 0, //
            0, 0, 0, 1, //
        };
        gizmo_mesh->draw(identity4x4, camera.state.viewProjection.data(), camera.state.position.data());

        //
        // present
        //
        renderer.endFrame();
        wgl.Present();
    }
    return EXIT_SUCCESS;
}
