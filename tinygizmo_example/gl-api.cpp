#include "gl-api.hpp"
#include <iostream>

#include <windows.h>

static bool gEnableGLDebugOutputErrorBreakpoints = false;

static void compile_shader(GLuint program, GLenum type, const char *source)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint status, length;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);

    if (status == GL_FALSE)
    {
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        std::vector<GLchar> buffer(length);
        glGetShaderInfoLog(shader, (GLsizei)buffer.size(), nullptr, buffer.data());
        glDeleteShader(shader);
        std::cerr << "GL Compile Error: " << buffer.data() << std::endl;
        std::cerr << "Source: " << source << std::endl;
        throw std::runtime_error("GLSL Compile Failure");
    }

    glAttachShader(program, shader);
    glDeleteShader(shader);
}

static std::string gl_src_to_str(GLenum source)
{
    switch (source)
    {
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
        return "WINDOW_SYSTEM";
    case GL_DEBUG_SOURCE_SHADER_COMPILER:
        return "SHADER_COMPILER";
    case GL_DEBUG_SOURCE_THIRD_PARTY:
        return "THIRD_PARTY";
    case GL_DEBUG_SOURCE_APPLICATION:
        return "APPLICATION";
    case GL_DEBUG_SOURCE_OTHER:
        return "OTHER";
    case GL_DEBUG_SOURCE_API:
        return "API";
    default:
        return "UNKNOWN";
    }
}

static std::string gl_enum_to_str(GLenum type)
{
    switch (type)
    {
    case GL_DEBUG_TYPE_ERROR:
        return "ERROR";
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
        return "DEPRECATION";
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
        return "UNDEFINED_BEHAVIOR";
    case GL_DEBUG_TYPE_PORTABILITY:
        return "PORTABILITY";
    case GL_DEBUG_TYPE_PERFORMANCE:
        return "PERFORMANCE";
    case GL_DEBUG_TYPE_OTHER:
        return "OTHER";
    default:
        return "UNKNOWN";
    }
}

static std::string gl_severity_to_str(GLenum severity)
{
    switch (severity)
    {
    case GL_DEBUG_SEVERITY_LOW:
        return "LOW";
    case GL_DEBUG_SEVERITY_MEDIUM:
        return "MEDIUM";
    case GL_DEBUG_SEVERITY_HIGH:
        return "HIGH";
    default:
        return "UNKNOWN";
    }
}

static void APIENTRY gl_debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, GLvoid *userParam)
{
    if (type != GL_DEBUG_TYPE_ERROR)
        return;
    auto sourceStr = gl_src_to_str(source);
    auto typeStr = gl_enum_to_str(type);
    auto severityStr = gl_severity_to_str(severity);
    std::cout << "gl_debug_callback: " << sourceStr << ", " << severityStr << ", " << typeStr << " , " << id << ", " << message << std::endl;
    if ((type == GL_DEBUG_TYPE_ERROR) && (gEnableGLDebugOutputErrorBreakpoints))
        __debugbreak();
}

void gl_check_error(const char *file, int32_t line)
{
#if defined(_DEBUG) || defined(DEBUG)
    GLint error = glGetError();
    if (error)
    {
        const char *errorStr = 0;
        switch (error)
        {
        case GL_INVALID_ENUM:
            errorStr = "GL_INVALID_ENUM";
            break;
        case GL_INVALID_VALUE:
            errorStr = "GL_INVALID_VALUE";
            break;
        case GL_INVALID_OPERATION:
            errorStr = "GL_INVALID_OPERATION";
            break;
        case GL_OUT_OF_MEMORY:
            errorStr = "GL_OUT_OF_MEMORY";
            break;
        default:
            errorStr = "unknown error";
            break;
        }
        printf("GL error : %s, line %d : %s\n", file, line, errorStr);
        error = 0;
    }
#endif
}

static size_t gl_size_bytes(GLenum type)
{
    switch (type)
    {
    case GL_UNSIGNED_BYTE:
        return sizeof(uint8_t);
    case GL_UNSIGNED_SHORT:
        return sizeof(uint16_t);
    case GL_UNSIGNED_INT:
        return sizeof(uint32_t);
    default:
        throw std::logic_error("unknown element type");
        break;
    }
}

void GlTexture2D::setup(GLsizei width, GLsizei height, GLenum internal_fmt, GLenum format, GLenum type, const GLvoid *pixels, bool createMipmap)
{
    glTextureImage2DEXT(*this, GL_TEXTURE_2D, 0, internal_fmt, width, height, 0, format, type, pixels);
    if (createMipmap)
        glGenerateTextureMipmapEXT(*this, GL_TEXTURE_2D);
    glTextureParameteriEXT(*this, GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTextureParameteriEXT(*this, GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, createMipmap ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTextureParameteriEXT(*this, GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTextureParameteriEXT(*this, GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    this->width = width;
    this->height = height;
}

void GlTexture3D::setup(GLenum target, GLsizei width, GLsizei height, GLsizei depth, GLenum internal_fmt, GLenum format, GLenum type, const GLvoid *pixels)
{
    glTextureImage3DEXT(*this, target, 0, internal_fmt, width, height, depth, 0, format, type, pixels);
    glTextureParameteriEXT(*this, target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTextureParameteriEXT(*this, target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTextureParameteriEXT(*this, target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTextureParameteriEXT(*this, target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTextureParameteriEXT(*this, target, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    this->width = width;
    this->height = height;
    this->depth = depth;
}

GlShader::GlShader(const GLuint type, const std::string &src)
{
    program = glCreateProgram();

    ::compile_shader(program, type, src.c_str());
    glProgramParameteri(program, GL_PROGRAM_SEPARABLE, GL_TRUE);

    glLinkProgram(program);

    GLint status, length;
    glGetProgramiv(program, GL_LINK_STATUS, &status);

    if (status == GL_FALSE)
    {
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
        std::vector<GLchar> buffer(length);
        glGetProgramInfoLog(program, (GLsizei)buffer.size(), nullptr, buffer.data());
        std::cerr << "GL Link Error: " << buffer.data() << std::endl;
        throw std::runtime_error("GLSL Link Failure");
    }
}

GlShader::GlShader(const std::string &vert, const std::string &frag, const std::string &geom)
{
    program = glCreateProgram();

    glProgramParameteri(program, GL_PROGRAM_SEPARABLE, GL_FALSE);

    ::compile_shader(program, GL_VERTEX_SHADER, vert.c_str());
    ::compile_shader(program, GL_FRAGMENT_SHADER, frag.c_str());

    if (geom.length() != 0)
        ::compile_shader(program, GL_GEOMETRY_SHADER, geom.c_str());

    glLinkProgram(program);

    GLint status, length;
    glGetProgramiv(program, GL_LINK_STATUS, &status);

    if (status == GL_FALSE)
    {
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
        std::vector<GLchar> buffer(length);
        glGetProgramInfoLog(program, (GLsizei)buffer.size(), nullptr, buffer.data());
        std::cerr << "GL Link Error: " << buffer.data() << std::endl;
        throw std::runtime_error("GLSL Link Failure");
    }
}

std::map<uint32_t, std::string> GlShader::reflect()
{
    std::map<uint32_t, std::string> locations;
    GLint count;
    glGetProgramiv(program, GL_ACTIVE_UNIFORMS, &count);
    for (GLuint i = 0; i < static_cast<GLuint>(count); ++i)
    {
        char buffer[1024];
        GLenum type;
        GLsizei length;
        GLint size, block_index;
        glGetActiveUniform(program, i, sizeof(buffer), &length, &size, &type, buffer);
        glGetActiveUniformsiv(program, 1, &i, GL_UNIFORM_BLOCK_INDEX, &block_index);
        if (block_index != -1)
            continue;
        GLint loc = glGetUniformLocation(program, buffer);
        locations[loc] = std::string(buffer);
    }
    return locations;
}

void GlShader::texture(GLint loc, GLenum target, int unit, GLuint tex) const
{
    glBindMultiTextureEXT(GL_TEXTURE0 + unit, target, tex);
    glProgramUniform1i(program, loc, unit);
}

void GlMesh::draw_elements(int instances) const
{
    if (vertexBuffer.size)
    {
        glBindVertexArray(vao);
        if (indexCount)
        {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
            if (instances)
                glDrawElementsInstanced(drawMode, indexCount, indexType, 0, instances);
            else
                glDrawElements(drawMode, indexCount, indexType, nullptr);
        }
        else
        {
            if (instances)
                glDrawArraysInstanced(drawMode, 0, static_cast<GLsizei>(vertexBuffer.size / vertexStride), instances);
            else
                glDrawArrays(drawMode, 0, static_cast<GLsizei>(vertexBuffer.size / vertexStride));
        }
        glBindVertexArray(0);
    }
}

void GlMesh::set_index_data(GLenum mode, GLenum type, GLsizei count, const GLvoid *data, GLenum usage)
{
    size_t size = gl_size_bytes(type);
    indexBuffer.set_buffer_data(size * count, data, usage);
    drawMode = mode;
    indexType = type;
    indexCount = count;
}
