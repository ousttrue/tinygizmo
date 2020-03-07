#include "renderer.h"
#include "dx11_context.h"
#include "teapot.h"
#include <DirectXMath.h>
#include <wrl/client.h>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <array>

template <class T>
using ComPtr = Microsoft::WRL::ComPtr<T>;

struct Vertex
{
    DirectX::XMFLOAT3 position;
    DirectX::XMFLOAT3 normal;
    DirectX::XMFLOAT4 color;
};

constexpr const char gizmo_shader[] = R"(
    struct VS_INPUT
	{
		float3 position : POSITION;
        float3 normal   : NORMAL;
		float4 color    : COLOR0;
	};
    struct VS_OUTPUT
    {
        linear float4 position: SV_POSITION;
        linear float3 normal  : NORMAL;
        linear float4 color   : COLOR0;
        linear float3 world   : POSITION;
    };    
	cbuffer cbContextData : register(b0)
	{
		float4x4 uModel;
		float4x4 uViewProj;
        float3 uEye;
	};
    
    VS_OUTPUT vsMain(VS_INPUT _in) 
    {
        VS_OUTPUT ret;
        ret.world = mul(uModel, float4(_in.position, 1)).xyz;
        ret.position = mul(uViewProj, float4(ret.world, 1));
        ret.normal = _in.normal;
        ret.color = _in.color;
        return ret;
    }

	float4 psMain(VS_OUTPUT _in): SV_Target
    {
        float3 light = float3(1, 1, 1) * max(dot(_in.normal, normalize(uEye - _in.world)), 0.50) + 0.25;
        return _in.color * float4(light, 1);
    }
)";

constexpr const char lit_shader[] = R"(
    struct VS_INPUT
	{
		float3 position : POSITION;
        float3 normal   : NORMAL;
		float4 color    : COLOR0;
	};
    struct VS_OUTPUT
    {
        linear float4 position: SV_POSITION;
        linear float3 normal  : NORMAL;
        linear float4 color   : COLOR0;
        linear float3 world   : POSITION;
    };    
	cbuffer cbContextData : register(b0)
	{
		float4x4 uModel;
		float4x4 uViewProj;
        float3 uEye;
	};

    VS_OUTPUT vsMain(VS_INPUT _in) 
    {
        VS_OUTPUT ret;
        ret.world = mul(uModel, float4(_in.position, 1)).xyz;
        ret.position = mul(uViewProj, float4(ret.world, 1));
        ret.normal = _in.normal;
        ret.color = _in.color;
        return ret;
    }

	float4 psMain(VS_OUTPUT _in): SV_Target
    {
        float3 light = float3(0.5, 0.3, 0.3) * max(dot(_in.normal, normalize(uEye - _in.world)), 0.50) + 0.25;
        // return _in.color * float4(light, 1);
        return float4(light, 1);
    }
)";

static ComPtr<ID3DBlob> LoadCompileShader(const std::string &src, const char *name, const char *entryPoint, const D3D_SHADER_MACRO *define, const char *target)
{
    UINT flags = D3DCOMPILE_ENABLE_STRICTNESS;

    ComPtr<ID3DBlob> ret;
    ComPtr<ID3DBlob> err;
    if (FAILED(D3DCompile(src.data(), src.size(), name, define, nullptr, entryPoint, target, flags, 0, &ret, &err)))
    {
        auto error = (char *)err->GetBufferPointer();
        std::cerr << name << ": " << error << std::endl;
        std::cerr << src << std::endl;
        return nullptr;
    }
    return ret;
}

struct ConstantBuffer
{
    std::array<float, 16> model;
    std::array<float, 16> viewProjection;
    std::array<float, 3> eye;
    float padding;
};
static_assert(sizeof(ConstantBuffer) == sizeof(float) * (16 * 2 + 4));

class Shader
{
    ComPtr<ID3D11VertexShader> m_vs;
    ComPtr<ID3D11PixelShader> m_ps;
    ComPtr<ID3D11InputLayout> m_inputLayout;
    ComPtr<ID3D11Buffer> m_cb;
    ComPtr<ID3D11RasterizerState> m_rasterizerState;

public:
    bool initialize(ID3D11Device *device,
                    const std::string &vs, const std::string &vsEntryPoint,
                    const std::string &ps, const std::string &psEntryPoint)
    {
        auto vsBlob = LoadCompileShader(vs, "vs", vsEntryPoint.c_str(), nullptr, "vs_4_0");
        if (!vsBlob)
        {
            return false;
        }
        if (FAILED(device->CreateVertexShader((DWORD *)vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), nullptr, &m_vs)))
        {
            return false;
        }

        D3D11_INPUT_ELEMENT_DESC inputDesc[] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
            {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
            {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0},
        };
        if (FAILED(device->CreateInputLayout(inputDesc, _countof(inputDesc), vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), &m_inputLayout)))
        {
            return false;
        }

        auto psBlob = LoadCompileShader(ps, "ps", psEntryPoint.c_str(), nullptr, "ps_4_0");
        if (!psBlob)
        {
            return false;
        }
        if (FAILED(device->CreatePixelShader((DWORD *)psBlob->GetBufferPointer(), psBlob->GetBufferSize(), nullptr, &m_ps)))
        {
            return false;
        }

        {
            D3D11_BUFFER_DESC desc = {
                .ByteWidth = sizeof(ConstantBuffer),
                .Usage = D3D11_USAGE_DEFAULT,
                .BindFlags = D3D11_BIND_CONSTANT_BUFFER,
            };
            // desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            if (FAILED(device->CreateBuffer(&desc, nullptr, &m_cb)))
            {
                return false;
            }
        }

        {
            D3D11_RASTERIZER_DESC rasterizerDesc = {
                .FillMode = D3D11_FILL_SOLID,
                .CullMode = D3D11_CULL_BACK,
                .FrontCounterClockwise = true,
            };
            if (FAILED(device->CreateRasterizerState(&rasterizerDesc, &m_rasterizerState)))
            {
                return false;
            }
        }

        return true;
    }

    void setup(ID3D11DeviceContext *context,
               const std::array<float, 3> &eye,
               const std::array<float, 16> &viewProj,
               const std::array<float, 16> &model)
    {
        if (!m_vs)
        {
            return;
        }
        ConstantBuffer data{
            .model = model,
            .viewProjection = viewProj,
            .eye = eye,
        };
        context->UpdateSubresource(m_cb.Get(), 0, nullptr, &data, 0, 0);

        ID3D11Buffer *cb_list[] = {m_cb.Get()};
        context->IASetInputLayout(m_inputLayout.Get());
        context->VSSetShader(m_vs.Get(), nullptr, 0);
        context->VSSetConstantBuffers(0, _countof(cb_list), cb_list);
        context->PSSetShader(m_ps.Get(), nullptr, 0);
        context->PSSetConstantBuffers(0, _countof(cb_list), cb_list);

        context->OMSetBlendState(nullptr, nullptr, 0xffffffff);
        context->OMSetDepthStencilState(nullptr, 0);
        context->RSSetState(m_rasterizerState.Get());
    }
};

class ModelImpl
{
    std::unique_ptr<Shader> m_shader;

    ComPtr<ID3D11Buffer> m_vb;

    ComPtr<ID3D11Buffer> m_ib;
    DXGI_FORMAT m_indexFormat = DXGI_FORMAT_R32_UINT;
    int m_indexCount = 0;

public:
    void upload_dynamic_mesh(ID3D11Device *device,
                             const uint8_t *pVertices, uint32_t verticesBytes, uint32_t vertexStride,
                             const uint8_t *pIndices, uint32_t indicesBytes, uint32_t indexStride)
    {
        ComPtr<ID3D11DeviceContext> context;
        device->GetImmediateContext(&context);

        if (!m_shader)
        {
            m_shader.reset(new Shader);
            if (!m_shader->initialize(device,
                                      gizmo_shader, "vsMain",
                                      gizmo_shader, "psMain"))
            {
                return;
            }
        }
        if (!m_vb)
        {
            D3D11_BUFFER_DESC desc{
                .ByteWidth = sizeof(Vertex) * 65535,
                .Usage = D3D11_USAGE_DYNAMIC,
                .BindFlags = D3D11_BIND_VERTEX_BUFFER,
                .CPUAccessFlags = D3D11_CPU_ACCESS_WRITE,
            };
            if (FAILED(device->CreateBuffer(&desc, nullptr, &m_vb)))
            {
                return;
            }
        }
        {
            D3D11_MAPPED_SUBRESOURCE mapped;
            context->Map(m_vb.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped);
            memcpy(mapped.pData, pVertices, verticesBytes);
            context->Unmap(m_vb.Get(), 0);
        }

        m_indexCount = indicesBytes / indexStride;
        if (!m_ib)
        {
            switch (indexStride)
            {
            case 4:
                m_indexFormat = DXGI_FORMAT_R32_UINT;
                break;
            case 2:
                m_indexFormat = DXGI_FORMAT_R16_UINT;
                break;
            default:
                throw;
            }
            D3D11_BUFFER_DESC desc = {
                .ByteWidth = indexStride * 65535,
                .Usage = D3D11_USAGE_DYNAMIC,
                .BindFlags = D3D11_BIND_INDEX_BUFFER,
                .CPUAccessFlags = D3D11_CPU_ACCESS_WRITE,
            };
            if (FAILED(device->CreateBuffer(&desc, nullptr, &m_ib)))
            {
                return;
            }
        }
        {
            D3D11_MAPPED_SUBRESOURCE mapped;
            context->Map(m_ib.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped);
            memcpy(mapped.pData, pIndices, indicesBytes);
            context->Unmap(m_ib.Get(), 0);
        }
    }

    void upload_static_mesh(ID3D11Device *device,
                            const uint8_t *pVertices, uint32_t verticesBytes, uint32_t vertexStride,
                            const uint8_t *pIndices, uint32_t indicesBytes, uint32_t indexStride)
    {
        if (!m_shader)
        {
            m_shader.reset(new Shader);
            if (!m_shader->initialize(device,
                                      lit_shader, "vsMain",
                                      lit_shader, "psMain"))
            {
                return;
            }
        }
        if (!m_vb)
        {
            D3D11_BUFFER_DESC desc{
                .ByteWidth = verticesBytes,
                .Usage = D3D11_USAGE_IMMUTABLE,
                .BindFlags = D3D11_BIND_VERTEX_BUFFER,
            };
            D3D11_SUBRESOURCE_DATA subRes = {
                .pSysMem = pVertices,
            };
            if (FAILED(device->CreateBuffer(&desc, &subRes, &m_vb)))
            {
                return;
            }
        }
        if (!m_ib)
        {
            m_indexCount = indicesBytes / indexStride;
            switch (indexStride)
            {
            case 4:
                m_indexFormat = DXGI_FORMAT_R32_UINT;
                break;
            case 2:
                m_indexFormat = DXGI_FORMAT_R16_UINT;
                break;
            default:
                throw;
            }
            D3D11_BUFFER_DESC desc = {
                .ByteWidth = indicesBytes,
                .Usage = D3D11_USAGE_IMMUTABLE,
                .BindFlags = D3D11_BIND_INDEX_BUFFER,
            };
            D3D11_SUBRESOURCE_DATA subRes = {
                .pSysMem = pIndices,
            };
            if (FAILED(device->CreateBuffer(&desc, &subRes, &m_ib)))
            {
                return;
            }
        }
    }

    void draw(ID3D11DeviceContext *context,
              const std::array<float, 3> &eye,
              const std::array<float, 16> &viewProj,
              const std::array<float, 16> &model)
    {
        if (!m_shader)
        {
            return;
        }
        if (!m_vb || !m_ib)
        {
            return;
        }

        m_shader->setup(context, eye, viewProj, model);

        ID3D11Buffer *vbs[]{m_vb.Get()};
        UINT strides[]{sizeof(Vertex)};
        UINT offsets[]{0};
        context->IASetVertexBuffers(0, 1, vbs, strides, offsets);
        context->IASetIndexBuffer(m_ib.Get(), m_indexFormat, 0);
        context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        context->DrawIndexed(m_indexCount, 0, 0);
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

void Model::uploadMesh(void *device,
                       const void *vertices, uint32_t verticesSize, uint32_t vertexStride,
                       const void *indices, uint32_t indicesSize, uint32_t indexSize,
                       bool is_dynamic)
{
    if (is_dynamic)
    {
        m_impl->upload_dynamic_mesh((ID3D11Device *)device,
                                    (const uint8_t *)vertices, verticesSize, vertexStride,
                                    (const uint8_t *)indices, indicesSize, indexSize);
    }
    else
    {
        m_impl->upload_static_mesh((ID3D11Device *)device,
                                   (const uint8_t *)vertices, verticesSize, vertexStride,
                                   (const uint8_t *)indices, indicesSize, indexSize);
    }
}

void Model::draw(void *context, const float *model, const float *vp, const float *eye)
{
    m_impl->draw((ID3D11DeviceContext *)context,
                 *(std::array<float, 3> *)eye,
                 *(std::array<float, 16> *)vp,
                 *(std::array<float, 16> *)model);
}

/////////////////////////////////////////////
class RendererImpl
{
    DX11Context m_d3d11;
    int m_width = 0;
    int m_height = 0;

public:
    void *initialize(void *hwnd)
    {
        return m_d3d11.Create(hwnd);
    }

    void *beginFrame(int width, int height)
    {
        return m_d3d11.NewFrame(width, height);
    }

    void clearDepth()
    {
        m_d3d11.ClearDepth();
    }

    void endFrame()
    {
        m_d3d11.Present();
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
    return m_impl->initialize(hwnd);
}

std::shared_ptr<Model> Renderer::createMesh()
{
    auto modelImpl = new ModelImpl();
    return std::make_shared<Model>(modelImpl);
}

void *Renderer::beginFrame(int width, int height)
{
    return m_impl->beginFrame(width, height);
}

void Renderer::endFrame()
{
    m_impl->endFrame();
}

void Renderer::clearDepth()
{
    m_impl->clearDepth();
}
