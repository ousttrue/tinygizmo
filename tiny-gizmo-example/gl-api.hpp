// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org>

#pragma once

#ifndef gl_api_hpp
#define gl_api_hpp

#define GLEW_STATIC
#define GL_GLEXT_PROTOTYPES
#include "glew.h"

#include <map>
#include <cstring>
#include <string>
#include <vector>
#include "linalg.h"

void gl_check_error(const char *file, int32_t line);

template <typename factory_t>
class GlObject
{
    mutable GLuint handle = 0;
    std::string n;

public:
    GlObject() {}
    GlObject(GLuint h) : handle(g) {}
    ~GlObject()
    {
        if (handle)
            factory_t::destroy(handle);
    }
    GlObject(const GlObject &r) = delete;
    GlObject &operator=(GlObject &&r)
    {
        std::swap(handle, r.handle);
        std::swap(n, r.n);
        return *this;
    }
    GlObject(GlObject &&r) { *this = std::move(r); }
    operator GLuint() const
    {
        if (!handle)
            factory_t::create(handle);
        return handle;
    }
    GlObject &operator=(GLuint &other)
    {
        handle = other;
        return *this;
    }
    void set_name(const std::string &newName) { n = newName; }
    std::string name() const { return n; }
    GLuint id() const { return handle; };
};

struct GlBufferFactory
{
    static void create(GLuint &x) { glGenBuffers(1, &x); };
    static void destroy(GLuint x) { glDeleteBuffers(1, &x); };
};
struct GlTextureFactory
{
    static void create(GLuint &x) { glGenTextures(1, &x); };
    static void destroy(GLuint x) { glDeleteTextures(1, &x); };
};
struct GlVertexArrayFactory
{
    static void create(GLuint &x) { glGenVertexArrays(1, &x); };
    static void destroy(GLuint x) { glDeleteVertexArrays(1, &x); };
};
struct GlRenderbufferFactory
{
    static void create(GLuint &x) { glGenRenderbuffers(1, &x); };
    static void destroy(GLuint x) { glDeleteRenderbuffers(1, &x); };
};
struct GlFramebufferFactory
{
    static void create(GLuint &x) { glGenFramebuffers(1, &x); };
    static void destroy(GLuint x) { glDeleteFramebuffers(1, &x); };
};
struct GlQueryFactory
{
    static void create(GLuint &x) { glGenQueries(1, &x); };
    static void destroy(GLuint x) { glDeleteQueries(1, &x); };
};
struct GlSamplerFactory
{
    static void create(GLuint &x) { glGenSamplers(1, &x); };
    static void destroy(GLuint x) { glDeleteSamplers(1, &x); };
};
struct GlTransformFeedbacksFactory
{
    static void create(GLuint &x) { glGenTransformFeedbacks(1, &x); };
    static void destroy(GLuint x) { glDeleteTransformFeedbacks(1, &x); };
};

typedef GlObject<GlBufferFactory> GlBufferObject;
typedef GlObject<GlTextureFactory> GlTextureObject;
typedef GlObject<GlVertexArrayFactory> GlVertexArrayObject;
typedef GlObject<GlRenderbufferFactory> GlRenderbufferObject;
typedef GlObject<GlFramebufferFactory> GlFramebufferObject;
typedef GlObject<GlQueryFactory> GlQueryObject;
typedef GlObject<GlSamplerFactory> GlSamplerObject;
typedef GlObject<GlTransformFeedbacksFactory> GlTransformFeedbacksObject;

//////////////////
//   GlBuffer   //
//////////////////

struct GlBuffer : public GlBufferObject
{
    GLsizeiptr size = 0;
    GlBuffer() {}
    void set_buffer_data(const GLsizeiptr s, const GLvoid *data, const GLenum usage)
    {
        this->size = s;
        glNamedBufferDataEXT(*this, size, data, usage);
    }
    void set_buffer_data(const std::vector<GLubyte> &bytes, const GLenum usage) { set_buffer_data(bytes.size(), bytes.data(), usage); }
    void set_buffer_sub_data(const GLsizeiptr s, const GLintptr offset, const GLvoid *data) { glNamedBufferSubDataEXT(*this, offset, s, data); }
    void set_buffer_sub_data(const std::vector<GLubyte> &bytes, const GLintptr offset, const GLenum usage) { set_buffer_sub_data(bytes.size(), offset, bytes.data()); }
};

////////////////////////
//   GlRenderbuffer   //
////////////////////////

struct GlRenderbuffer : public GlRenderbufferObject
{
    float width{0}, height{0};
    GlRenderbuffer() {}
    GlRenderbuffer(float width, float height) : width(width), height(height) {}
};

///////////////////////
//   GlFramebuffer   //
///////////////////////

struct GlFramebuffer : public GlFramebufferObject
{
    float width{0}, height{0}, depth{0};
    GlFramebuffer() {}
    GlFramebuffer(float width, float height) : width(width), height(height) {}
    GlFramebuffer(float width, float height, float depth) : width(width), height(height), depth(depth) {}
    void check_complete()
    {
        if (glCheckNamedFramebufferStatusEXT(*this, GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            throw std::runtime_error("fbo incomplete");
    }
};

///////////////////
//   GlTexture   //
///////////////////

struct GlTexture2D : public GlTextureObject
{
    float width{0}, height{0};
    GlTexture2D() {}
    GlTexture2D(float width, float height) : width(width), height(height) {}
    void setup(GLsizei width, GLsizei height, GLenum internal_fmt, GLenum format, GLenum type, const GLvoid *pixels, bool createMipmap = false);
};

/////////////////////
//   GlTexture3D   //
/////////////////////

// As either a 3D texture or 2D array
struct GlTexture3D : public GlTextureObject
{
    float width{0}, height{0}, depth{0};
    GlTexture3D() {}
    GlTexture3D(float width, float height, float depth) : width(width), height(height), depth(depth) {}
    void setup(GLenum target, GLsizei width, GLsizei height, GLsizei depth, GLenum internal_fmt, GLenum format, GLenum type, const GLvoid *pixels);
};

//////////////////
//   GlShader   //
//////////////////

class GlShader
{
    GLuint program;
    bool enabled = false;

protected:
    GlShader(const GlShader &r) = delete;
    GlShader &operator=(const GlShader &r) = delete;

public:
    GlShader() : program() {}
    GlShader(const GLuint type, const std::string &src);
    GlShader(const std::string &vert, const std::string &frag, const std::string &geom = "");
    GlShader(GlShader &&r) : GlShader() { *this = std::move(r); }
    ~GlShader()
    {
        if (program)
            glDeleteProgram(program);
    }

    GLuint handle() const { return program; }
    GLint get_uniform_location(const std::string &name) const { return glGetUniformLocation(program, name.c_str()); }

    GlShader &operator=(GlShader &&r)
    {
        std::swap(program, r.program);
        return *this;
    }

    std::map<uint32_t, std::string> reflect();

    void uniform(const std::string &name, int scalar) const { glProgramUniform1i(program, get_uniform_location(name), scalar); }
    void uniform(const std::string &name, float scalar) const { glProgramUniform1f(program, get_uniform_location(name), scalar); }
    void uniform(const std::string &name, const linalg::aliases::float2 &vec) const { glProgramUniform2fv(program, get_uniform_location(name), 1, &vec.x); }
    void uniform(const std::string &name, const linalg::aliases::float3 &vec) const { glProgramUniform3fv(program, get_uniform_location(name), 1, &vec.x); }
    void uniform_float3(const std::string &name, const float *vec) const { glProgramUniform3fv(program, get_uniform_location(name), 1, vec); }
    void uniform(const std::string &name, const linalg::aliases::float4 &vec) const { glProgramUniform4fv(program, get_uniform_location(name), 1, &vec.x); }
    void uniform(const std::string &name, const linalg::aliases::float3x3 &mat) const { glProgramUniformMatrix3fv(program, get_uniform_location(name), 1, GL_FALSE, &mat.x.x); }
    void uniform(const std::string &name, const linalg::aliases::float4x4 &mat) const { glProgramUniformMatrix4fv(program, get_uniform_location(name), 1, GL_FALSE, &mat.x.x); }

    void uniform(const std::string &name, const int elements, const std::vector<int> &scalar) const { glProgramUniform1iv(program, get_uniform_location(name), elements, scalar.data()); }
    void uniform(const std::string &name, const int elements, const std::vector<float> &scalar) const { glProgramUniform1fv(program, get_uniform_location(name), elements, scalar.data()); }
    void uniform(const std::string &name, const int elements, const std::vector<linalg::aliases::float2> &vec) const { glProgramUniform2fv(program, get_uniform_location(name), elements, &vec[0].x); }
    void uniform(const std::string &name, const int elements, const std::vector<linalg::aliases::float3> &vec) const { glProgramUniform3fv(program, get_uniform_location(name), elements, &vec[0].x); }
    void uniform(const std::string &name, const int elements, const std::vector<linalg::aliases::float3x3> &mat) const { glProgramUniformMatrix3fv(program, get_uniform_location(name), elements, GL_FALSE, &mat[0].x.x); }
    void uniform(const std::string &name, const int elements, const std::vector<linalg::aliases::float4x4> &mat) const { glProgramUniformMatrix4fv(program, get_uniform_location(name), elements, GL_FALSE, &mat[0].x.x); }

    void texture(GLint loc, GLenum target, int unit, GLuint tex) const;
    void texture(const char *name, int unit, GLuint tex, GLenum target) const { texture(get_uniform_location(name), target, unit, tex); }

    void bind()
    {
        if (program > 0)
            enabled = true;
        glUseProgram(program);
    }
    void unbind()
    {
        enabled = false;
        glUseProgram(0);
    }
};

////////////////
//   GlMesh   //
////////////////

class GlMesh
{
    GlVertexArrayObject vao;
    GlBuffer vertexBuffer, instanceBuffer, indexBuffer;

    GLenum drawMode = GL_TRIANGLES;
    GLenum indexType = 0;
    GLsizei vertexStride = 0, instanceStride = 0, indexCount = 0;

public:
    GlMesh() {}
    GlMesh(GlMesh &&r) { *this = std::move(r); }
    GlMesh(const GlMesh &r) = delete;
    GlMesh &operator=(GlMesh &&r)
    {
        char buffer[sizeof(GlMesh)];
        memcpy(buffer, this, sizeof(buffer));
        memcpy(this, &r, sizeof(buffer));
        memcpy(&r, buffer, sizeof(buffer));
        return *this;
    }
    GlMesh &operator=(const GlMesh &r) = delete;
    ~GlMesh(){};

    void set_non_indexed(GLenum newMode)
    {
        drawMode = newMode;
        indexBuffer = {};
        indexType = 0;
        indexCount = 0;
    }

    void draw_elements(int instances = 0) const;

    void set_vertex_data(GLsizeiptr size, const GLvoid *data, GLenum usage) { vertexBuffer.set_buffer_data(size, data, usage); }
    GlBuffer &get_vertex_data_buffer() { return vertexBuffer; };

    void set_instance_data(GLsizeiptr size, const GLvoid *data, GLenum usage) { instanceBuffer.set_buffer_data(size, data, usage); }

    void set_index_data(GLenum mode, GLenum type, GLsizei count, const GLvoid *data, GLenum usage);

    GlBuffer &get_index_data_buffer() { return indexBuffer; };

    void set_attribute(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid *offset)
    {
        glEnableVertexArrayAttribEXT(vao, index);
        glVertexArrayVertexAttribOffsetEXT(vao, vertexBuffer, index, size, type, normalized, stride, (GLintptr)offset);
        vertexStride = stride;
    }

    void set_instance_attribute(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid *offset)
    {
        glEnableVertexArrayAttribEXT(vao, index);
        glVertexArrayVertexAttribOffsetEXT(vao, instanceBuffer, index, size, type, normalized, stride, (GLintptr)offset);
        glVertexArrayVertexAttribDivisorEXT(vao, index, 1);
        instanceStride = stride;
    }

    void set_indices(GLenum mode, GLsizei count, const uint8_t *indices, GLenum usage) { set_index_data(mode, GL_UNSIGNED_BYTE, count, indices, usage); }
    void set_indices(GLenum mode, GLsizei count, const uint16_t *indices, GLenum usage) { set_index_data(mode, GL_UNSIGNED_SHORT, count, indices, usage); }
    void set_indices(GLenum mode, GLsizei count, const uint32_t *indices, GLenum usage) { set_index_data(mode, GL_UNSIGNED_INT, count, indices, usage); }

    template <class T>
    void set_vertices(size_t count, const T *vertices, GLenum usage) { set_vertex_data(count * sizeof(T), vertices, usage); }
    template <class T>
    void set_vertices(const std::vector<T> &vertices, GLenum usage) { set_vertices(vertices.size(), vertices.data(), usage); }
    template <class T, int N>
    void set_vertices(const T (&vertices)[N], GLenum usage) { set_vertices(N, vertices, usage); }

    template <class V>
    void set_attribute(GLuint index, float V::*field) { set_attribute(index, 1, GL_FLOAT, GL_FALSE, sizeof(V), &(((V *)0)->*field)); }
    template <class V, int N>
    void set_attribute(GLuint index, linalg::vec<float, N> V::*field) { set_attribute(index, N, GL_FLOAT, GL_FALSE, sizeof(V), &(((V *)0)->*field)); }

    template <class T>
    void set_elements(GLsizei count, const linalg::vec<T, 2> *elements, GLenum usage) { set_indices(GL_LINES, count * 2, &elements->x, usage); }
    template <class T>
    void set_elements(GLsizei count, const linalg::vec<T, 3> *elements, GLenum usage) { set_indices(GL_TRIANGLES, count * 3, &elements->x, usage); }
    template <class T>
    void set_elements(GLsizei count, const linalg::vec<T, 4> *elements, GLenum usage) { set_indices(GL_QUADS, count * 4, &elements->x, usage); }

    template <class T>
    void set_elements(const std::vector<T> &elements, GLenum usage) { set_elements((GLsizei)elements.size(), elements.data(), usage); }

    template <class T, int N>
    void set_elements(const T (&elements)[N], GLenum usage) { set_elements(N, elements, usage); }
};

struct GlModel
{
    std::shared_ptr<GlShader> shader;
    GlMesh mesh;

    template <typename V, typename I>
    void upload_mesh(
        uint32_t vertexCount, const V *pVertices,
        uint32_t indexCount, const I *pIndices,
        bool isDynamic = false)
    {
        upload_mesh(
            pVertices, vertexCount * sizeof(V), sizeof(V),
            pIndices, indexCount * sizeof(I), sizeof(I) / 3,
            isDynamic);
    }

    void upload_mesh(
        const void *pVertices, uint32_t verticesBytes, uint32_t vertexStride,
        const void *pIndices, uint32_t indicesBytes, uint32_t indexStride,
        bool isDynamic = false)
    {
        mesh.set_vertex_data(verticesBytes, pVertices, isDynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
        mesh.set_attribute(0, 3, GL_FLOAT, GL_FALSE, vertexStride, (GLvoid *)0);
        mesh.set_attribute(1, 3, GL_FLOAT, GL_FALSE, vertexStride, (GLvoid *)12);
        mesh.set_attribute(2, 4, GL_FLOAT, GL_FALSE, vertexStride, (GLvoid *)24);

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
        mesh.set_index_data(GL_TRIANGLES, type, indicesBytes / indexStride, pIndices, isDynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
    }

    void draw(const float *eye, const linalg::aliases::float4x4 &viewProj, const linalg::aliases::float4x4 &model, bool isGizmo = false)
    {
        if (isGizmo)
        {
            glClear(GL_DEPTH_BUFFER_BIT);
        }
        shader->bind();
        shader->uniform("u_viewProj", viewProj);
        shader->uniform("u_modelMatrix", model);
        shader->uniform_float3("u_eye", eye);
        mesh.draw_elements();
        shader->unbind();
    }
};

class GlRenderer
{
public:
    bool initialize()
    {
        if (GLenum err = glewInit())
        {
            // throw std::runtime_error(std::string("glewInit() failed - ") + (const char *)glewGetErrorString(err));
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

    void endFrame()
    {
        gl_check_error(__FILE__, __LINE__);
    }
};

#endif // end gl_api_hpp
