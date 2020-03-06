#pragma once
#include <memory>
#include <stdint.h>

class Model
{
    class ModelImpl *m_impl = nullptr;

public:
    Model(ModelImpl *impl);
    ~Model();
    void uploadMesh(
        const void *vertices, uint32_t verticesSize, uint32_t vertexStride,
        const void *indices, uint32_t indicesSize, uint32_t indexSize,
        bool is_local);
    void draw(const float *model, const float *vp, const float *eye);
};

class Renderer
{
    class RendererImpl *m_impl = nullptr;

public:
    Renderer();
    ~Renderer();
    bool initialize();
    std::shared_ptr<Model> createMeshForGizmo();
    std::shared_ptr<Model> createMesh();
    void beginFrame(int width, int height);
    void endFrame();
    void clearDepth();
};
