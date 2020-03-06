#include "renderer.h"
#include "teapot.h"

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

public:
    ModelImpl(bool isGizmo)
    {
        if (isGizmo)
        {
        }
        else
        {
        }
    }

    void upload_mesh(
        const void *pVertices, uint32_t verticesBytes, uint32_t vertexStride,
        const void *pIndices, uint32_t indicesBytes, uint32_t indexStride,
        bool isDynamic = false)
    {
    }

    void draw(const float *eye, const float *viewProj, const float *model, bool isGizmo = false)
    {
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

void Model::uploadMesh(
    const void *vertices, uint32_t verticesSize, uint32_t vertexStride,
    const void *indices, uint32_t indicesSize, uint32_t indexSize,
    bool is_local)
{
    m_impl->upload_mesh(
        vertices, verticesSize, vertexStride,
        indices, indicesSize, indexSize,
        is_local);
}

void Model::draw(const float *model, const float *vp, const float *eye)
{
    m_impl->draw(eye, vp, model);
}

/////////////////////////////////////////////
class RendererImpl
{
public:
    bool initialize()
    {
        return true;
    }

    void beginFrame(int width, int height)
    {
    }

    void clearDepth()
    {
    }

    void endFrame()
    {
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

bool Renderer::initialize()
{
    return m_impl->initialize();
}

std::shared_ptr<Model> Renderer::createMeshForGizmo()
{
    auto modelImpl = new ModelImpl(true);
    return std::make_shared<Model>(modelImpl);
}

std::shared_ptr<Model> Renderer::createMesh()
{
    auto modelImpl = new ModelImpl(false);
    return std::make_shared<Model>(modelImpl);
}

void Renderer::beginFrame(int width, int height)
{
    m_impl->beginFrame(width, height);
}

void Renderer::endFrame()
{
    m_impl->endFrame();
}

void Renderer::clearDepth()
{
    m_impl->clearDepth();
}
