#pragma once
#include <array>
#include <limits>
#include <math.h>

namespace fpalg
{
using float2 = std::array<float, 2>;
using float3 = std::array<float, 3>;
using float4 = std::array<float, 4>;

template <typename T, typename S>
inline const T &size_cast(const S &s)
{
    static_assert(sizeof(S) == sizeof(T), "must same size");
    return *((T *)&s);
}

template <typename T, typename S>
inline T &size_cast(S &s)
{
    static_assert(sizeof(S) == sizeof(T), "must same size");
    return *((T *)&s);
}

inline float3 operator-(const float3 &lhs)
{
    return {-lhs[0], -lhs[1], -lhs[2]};
}
inline float3 operator+(const float3 &lhs, const float3 &rhs)
{
    return {lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]};
}
inline float3 &operator+=(float3 &lhs, const float3 &rhs)
{
    lhs[0] += rhs[0];
    lhs[1] += rhs[1];
    lhs[2] += rhs[2];
    return lhs;
}
inline float3 operator-(const float3 &lhs, const float3 &rhs)
{
    return {lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2]};
}
inline float3 operator*(const float3 &lhs, float scalar)
{
    return {lhs[0] * scalar, lhs[1] * scalar, lhs[2] * scalar};
}
inline float3 Mul3(const float3 &lhs, const float3 &rhs)
{
    return {lhs[0] * rhs[0], lhs[1] * rhs[1], lhs[2] * rhs[2]};
}

inline float Dot(const float3 &lhs, const float3 &rhs)
{
    return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

inline float Length(const float3 &lhs)
{
    return std::sqrt(Dot(lhs, lhs));
}

inline float3 Normalize(const float3 &lhs)
{
    auto factor = 1.0f / Length(lhs);
    return {lhs[0] * factor, lhs[1] * factor, lhs[2] * factor};
}

inline float3 Cross(const float3 &lhs, const float3 &rhs)
{
    return {
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0]};
}

inline float Dot4(const float *row, const float *col, int step = 1)
{
    auto i = 0;
    auto a = row[0] * col[i];
    i += step;
    auto b = row[1] * col[i];
    i += step;
    auto c = row[2] * col[i];
    i += step;
    auto d = row[3] * col[i];
    auto value = a + b + c + d;
    return value;
}

inline std::array<float, 16> Mul(const float l[16], const float r[16])
{
    auto _11 = Dot4(&l[0], &r[0], 4);
    auto _12 = Dot4(&l[0], &r[1], 4);
    auto _13 = Dot4(&l[0], &r[2], 4);
    auto _14 = Dot4(&l[0], &r[3], 4);
    auto _21 = Dot4(&l[4], &r[0], 4);
    auto _22 = Dot4(&l[4], &r[1], 4);
    auto _23 = Dot4(&l[4], &r[2], 4);
    auto _24 = Dot4(&l[4], &r[3], 4);
    auto _31 = Dot4(&l[8], &r[0], 4);
    auto _32 = Dot4(&l[8], &r[1], 4);
    auto _33 = Dot4(&l[8], &r[2], 4);
    auto _34 = Dot4(&l[8], &r[3], 4);
    auto _41 = Dot4(&l[12], &r[0], 4);
    auto _42 = Dot4(&l[12], &r[1], 4);
    auto _43 = Dot4(&l[12], &r[2], 4);
    auto _44 = Dot4(&l[12], &r[3], 4);

    return std::array<float, 16>{
        _11,
        _12,
        _13,
        _14,
        _21,
        _22,
        _23,
        _24,
        _31,
        _32,
        _33,
        _34,
        _41,
        _42,
        _43,
        _44,
    };
}

inline std::array<float, 16> operator*(const std::array<float, 16> &lhs, const std::array<float, 16> &rhs)
{
    return Mul(lhs.data(), rhs.data());
}

inline void Transpose(std::array<float, 16> &m)
{
    std::swap(m[1], m[4]);
    std::swap(m[2], m[8]);
    std::swap(m[3], m[12]);
    std::swap(m[6], m[9]);
    std::swap(m[7], m[13]);
    std::swap(m[11], m[14]);
}

inline std::array<float, 16> IdentityMatrix()
{
    return std::array<float, 16>{
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        1,
    };
}

inline std::array<float, 16> YawMatrix(float yawRadians)
{
    auto ys = (float)sin(yawRadians);
    auto yc = (float)cos(yawRadians);
    std::array<float, 16> yaw = {
        yc,
        0,
        ys,
        0,

        0,
        1,
        0,
        0,

        -ys,
        0,
        yc,
        0,

        0,
        0,
        0,
        1,
    };
    return yaw;
}

inline std::array<float, 16> PitchMatrix(float pitchRadians)
{
    auto ps = (float)sin(pitchRadians);
    auto pc = (float)cos(pitchRadians);
    std::array<float, 16> pitch = {
        1,
        0,
        0,
        0,

        0,
        pc,
        ps,
        0,

        0,
        -ps,
        pc,
        0,

        0,
        0,
        0,
        1,
    };
    return pitch;
}

inline std::array<float, 16> TranslationMatrix(float x, float y, float z)
{
    std::array<float, 16> t = {
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        x,
        y,
        z,
        1,
    };
    return t;
}

inline double Cot(double value)
{
    return 1.0f / tan(value);
}

inline void PerspectiveRHGL(float projection[16], float fovYRadians, float aspectRatio, float zNear, float zFar)
{
    const float f = static_cast<float>(Cot(fovYRadians / 2.0));
    projection[0] = f / aspectRatio;
    projection[1] = 0.0f;
    projection[2] = 0.0f;
    projection[3] = 0.0f;

    projection[4] = 0.0f;
    projection[5] = f;
    projection[6] = 0.0f;
    projection[7] = 0.0f;

    projection[8] = 0.0f;
    projection[9] = 0.0f;
    projection[10] = (zNear + zFar) / (zNear - zFar);
    projection[11] = -1;

    projection[12] = 0.0f;
    projection[13] = 0.0f;
    projection[14] = (2 * zFar * zNear) / (zNear - zFar);
    projection[15] = 0.0f;
}

inline void PerspectiveRHDX(float projection[16], float fovYRadians, float aspectRatio, float zNear, float zFar)
{
    auto yScale = (float)Cot(fovYRadians / 2);
    auto xScale = yScale / aspectRatio;
    projection[0] = xScale;
    projection[1] = 0.0f;
    projection[2] = 0.0f;
    projection[3] = 0.0f;

    projection[4] = 0.0f;
    projection[5] = yScale;
    projection[6] = 0.0f;
    projection[7] = 0.0f;

    projection[8] = 0.0f;
    projection[9] = 0.0f;
    projection[10] = zFar / (zNear - zFar);
    projection[11] = -1;

    projection[12] = 0.0f;
    projection[13] = 0.0f;
    projection[14] = (zFar * zNear) / (zNear - zFar);
    projection[15] = 0.0f;
}

inline float4 QuaternionAxisAngle(const float3 &axis, float angle)
{
    auto angle_half = angle / 2;
    auto sin = std::sin(angle_half);
    auto cos = std::cos(angle_half);
    return {axis[0] * sin, axis[1] * sin, axis[2] * sin, cos};
}

inline float4 QuaternionMul(const float4 &lhs, const float4 &rhs)
{
    return {lhs[0] * rhs[3] + lhs[3] * rhs[0] + lhs[1] * rhs[2] - lhs[2] * rhs[1],
            lhs[1] * rhs[3] + lhs[3] * rhs[1] + lhs[2] * rhs[0] - lhs[0] * rhs[2],
            lhs[2] * rhs[3] + lhs[3] * rhs[2] + lhs[0] * rhs[1] - lhs[1] * rhs[0],
            lhs[3] * rhs[3] - lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2]};
}
// inline float4 operator*(const float4 &lhs, const float4 &rhs)
// {
//     float ax = lhs[0];
//     float ay = lhs[1];
//     float az = lhs[2];
//     float aw = lhs[3];
//     float bx = rhs[0];
//     float by = rhs[1];
//     float bz = rhs[2];
//     float bw = rhs[3];
//     return {
//         ax * bw + aw * bx + ay * bz - az * by,
//         ay * bw + aw * by + az * bx - ax * bz,
//         az * bw + aw * bz + ax * by - ay * bx,
//         aw * bw - ax * bx - ay * by - az * bz,
//     };
// }

inline float4 QuaternionConjugate(const float4 &v)
{
    return {-v[0], -v[1], -v[2], v[3]};
}

inline float3 QuaternionXDir(const float4 &v)
{
    auto x = v[0];
    auto y = v[1];
    auto z = v[2];
    auto w = v[3];
    return {w * w + x * x - y * y - z * z, (x * y + z * w) * 2, (z * x - y * w) * 2};
}

inline float3 QuaternionYDir(const float4 &v)
{
    auto x = v[0];
    auto y = v[1];
    auto z = v[2];
    auto w = v[3];
    return {(x * y - z * w) * 2, w * w - x * x + y * y - z * z, (y * z + x * w) * 2};
}

inline float3 QuaternionZDir(const float4 &v)
{
    auto x = v[0];
    auto y = v[1];
    auto z = v[2];
    auto w = v[3];
    return {(z * x + y * w) * 2, (y * z - x * w) * 2, w * w - x * x - y * y + z * z};
}

inline std::array<float, 16> QuaternionMatrix(const float4 &q)
{
    auto x = QuaternionXDir(q);
    auto y = QuaternionYDir(q);
    auto z = QuaternionZDir(q);
    return {
        x[0], x[1], x[2], 0,
        y[0], y[1], y[2], 0,
        z[0], z[1], z[2], 0,
        0, 0, 0, 1};
}

inline std::array<float, 16> ScaleMatrix(const float3 &s)
{
    return {
        s[0], 0, 0, 0,
        0, s[1], 0, 0,
        0, 0, s[2], 0,
        0, 0, 0, 1};
}

inline float3 QuaternionRotateFloat3(const float4 &q, const float3 &v)
{
    auto x = QuaternionXDir(q);
    auto y = QuaternionYDir(q);
    auto z = QuaternionZDir(q);
    return x * v[0] + y * v[1] + z * v[2];
}

struct Transform
{
    float3 position{0, 0, 0};
    float4 rotation{0, 0, 0, 1};

    std::array<float, 16> Matrix() const
    {
        auto r = QuaternionMatrix(rotation);
        r[12] = position[0];
        r[13] = position[1];
        r[14] = position[2];
        return r;
    }

    Transform Inverse() const
    {
        auto inv_r = QuaternionConjugate(rotation);
        auto inv_t = QuaternionRotateFloat3(inv_r, -position);
        return {inv_t, inv_r};
    }

    float3 ApplyPosition(const float3 &v) const
    {
        return QuaternionRotateFloat3(rotation, v) + position;
    }

    float3 ApplyDirection(const float3 &v) const
    {
        return QuaternionRotateFloat3(rotation, v);
    }
};

struct TRS
{
    union {
        Transform transform;
        struct
        {
            float3 position;
            float4 rotation;
        };
    };
    float3 scale;

    TRS()
        : position({0, 0, 0}), rotation({0, 0, 0, 1}), scale({1, 1, 1})
    {
    }

    std::array<float, 16> Matrix() const
    {
        return ScaleMatrix(scale) * transform.Matrix();
    }
};
static_assert(sizeof(TRS) == 40, "TRS");

struct Ray
{
    float3 origin;
    float3 direction;

    Ray(const float3 &_origin, const float3 &_direction)
        : origin(_origin), direction(_direction)
    {
    }

    template <typename T>
    Ray(const T &t)
    {
        static_assert(sizeof(T) == sizeof(Ray), "Ray::Ray()");
        auto &ray = size_cast<Ray>(t);
        origin = ray.origin;
        direction = ray.direction;
    }

    Ray ToLocal(const fpalg::Transform &t) const
    {
        auto toLocal = t.Inverse();
        return {
            toLocal.ApplyPosition(origin),
            fpalg::QuaternionRotateFloat3(toLocal.rotation, direction)};
    }

    float3 SetT(float t) const
    {
        return origin + direction * t;
    }

    Ray Transform(const fpalg::Transform &t) const
    {
        return {
            t.ApplyPosition(origin),
            t.ApplyDirection(direction),
        };
    }
};

struct Plane
{
    float3 normal;
    float3 pointOnPlane;

    Plane(const float3 &n, const float3 &point_on_plane)
        : normal(n), pointOnPlane(point_on_plane)
    {
    }
};

inline float operator>>(const Ray &ray, const Plane &plane)
{
    auto NV = Dot(plane.normal, ray.direction);
    if (NV == 0)
    {
        // not intersect
        return -std::numeric_limits<float>::infinity();
    }

    auto Q = plane.pointOnPlane - ray.origin;
    auto NQ = Dot(plane.normal, Q);
    return NQ / NV;
}

struct Triangle
{
    float3 v0;
    float3 v1;
    float3 v2;
};

// float intersect_ray_triangle(const ray &ray, const minalg::float3 &v0, const minalg::float3 &v1, const minalg::float3 &v2)
inline float operator>>(const Ray &ray, const Triangle &triangle)
{
    auto e1 = triangle.v1 - triangle.v0;
    auto e2 = triangle.v2 - triangle.v0;
    auto h = Cross(ray.direction, e2);
    auto a = Dot(e1, h);
    if (std::abs(a) == 0)
        return std::numeric_limits<float>::infinity();

    float f = 1 / a;
    auto s = ray.origin - triangle.v0;
    auto u = f * Dot(s, h);
    if (u < 0 || u > 1)
        return std::numeric_limits<float>::infinity();

    auto q = Cross(s, e1);
    auto v = f * Dot(ray.direction, q);
    if (v < 0 || u + v > 1)
        return std::numeric_limits<float>::infinity();

    auto t = f * Dot(e2, q);
    if (t < 0)
        return std::numeric_limits<float>::infinity();

    return t;
}

struct Matrix2x3
{
    float3 x;
    float3 y;

    float3 Apply(const float2 &value) const
    {
        return {
            x[0] * value[0] + y[0] * value[1],
            x[1] * value[0] + y[1] * value[1],
            x[2] * value[0] + y[2] * value[1],
        };
    }
};

} // namespace fpalg
