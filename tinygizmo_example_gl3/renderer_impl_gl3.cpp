#include "renderer.h"
#include "wgl_context.h"
#include "teapot.h"
#include "gl-api.hpp"

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

class ModelImpl
{
    std::unique_ptr<GlShader> m_shader;
    GlMesh m_mesh;

public:
    void upload_mesh(
        const void *pVertices, uint32_t verticesBytes, uint32_t vertexStride,
        const void *pIndices, uint32_t indicesBytes, uint32_t indexStride,
        bool isDynamic = false)
    {
        if (!m_shader)
        {
            if (isDynamic)
            {
                m_shader.reset(new GlShader(gizmo_vert, gizmo_frag));
            }
            else
            {
                m_shader.reset(new GlShader(lit_vert, lit_frag));
            }
        }

        m_mesh.set_vertex_data(verticesBytes, pVertices, isDynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
        m_mesh.set_attribute(0, 3, GL_FLOAT, GL_FALSE, vertexStride, (GLvoid *)0);
        m_mesh.set_attribute(1, 3, GL_FLOAT, GL_FALSE, vertexStride, (GLvoid *)12);
        m_mesh.set_attribute(2, 4, GL_FLOAT, GL_FALSE, vertexStride, (GLvoid *)24);

        GLenum type;
        switch (indexStride)
        {
        case 2:
            type = GL_UNSIGNED_SHORT;
            break;
        case 4:
            type = GL_UNSIGNED_INT;
            break;
        default:
            throw;
        }
        m_mesh.set_index_data(GL_TRIANGLES, type,
                              indicesBytes / indexStride, pIndices,
                              isDynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
    }

    void draw(const float *eye, const float *viewProj, const float *model, bool isGizmo = false)
    {
        if (isGizmo)
        {
            glClear(GL_DEPTH_BUFFER_BIT);
        }
        m_shader->bind();
        m_shader->uniform_float16("u_viewProj", viewProj);
        m_shader->uniform_float16("u_modelMatrix", model);
        m_shader->uniform_float3("u_eye", eye);
        m_mesh.draw_elements();
        m_shader->unbind();
    }
};

Model::Model(ModelImpl *impl)
    : m_impl(impl)
{
}

Model::~Model()
{
    delete m_impl;
}

void Model::uploadMesh(void *,
                       const void *vertices, uint32_t verticesSize, uint32_t vertexStride,
                       const void *indices, uint32_t indicesSize, uint32_t indexSize,
                       bool is_dynamic)
{
    m_impl->upload_mesh(
        vertices, verticesSize, vertexStride,
        indices, indicesSize, indexSize,
        is_dynamic);
}

void Model::draw(void *, const float *model, const float *vp, const float *eye)
{
    m_impl->draw(eye, vp, model);
}

/////////////////////////////////////////////
class RendererImpl
{
    WGLContext m_wgl;

public:
    bool initialize(void *hwnd)
    {
        if (!m_wgl.Create(hwnd, 3, 0))
        {
            return false;
        }
        return true;
    }

    void beginFrame(int width, int height)
    {
        glViewport(0, 0, width, height);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glClearColor(0.725f, 0.725f, 0.725f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void clearDepth()
    {
        glClear(GL_DEPTH_BUFFER_BIT);
    }

    void endFrame()
    {
        gl_check_error(__FILE__, __LINE__);
        m_wgl.Present();
    }
};

Renderer::Renderer()
    : m_impl(new RendererImpl)
{
}

Renderer::~Renderer()
{
    delete m_impl;
}

void *Renderer::initialize(void *hwnd)
{
    return m_impl->initialize(hwnd) ? this : nullptr;
}

std::shared_ptr<Model> Renderer::createMesh()
{
    auto modelImpl = new ModelImpl();
    return std::make_shared<Model>(modelImpl);
}

void *Renderer::beginFrame(int width, int height)
{
    m_impl->beginFrame(width, height);
    return nullptr;
}

void Renderer::endFrame()
{
    m_impl->endFrame();
}

void Renderer::clearDepth()
{
    m_impl->clearDepth();
}
