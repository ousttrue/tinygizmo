#pragma once
#include <array>
#include <math.h>

namespace fpalg
{
inline std::array<float, 3> operator-(const std::array<float, 3> &lhs)
{
    return {-lhs[0], -lhs[1], -lhs[2]};
}
inline std::array<float, 3> operator+(const std::array<float, 3> &lhs, const std::array<float, 3> &rhs)
{
    return {lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]};
}
inline std::array<float, 3> operator*(const std::array<float, 3> &lhs, float scalar)
{
    return {lhs[0] * scalar, lhs[1] * scalar, lhs[2] * scalar};
}

inline float Dot(const float *row, const float *col)
{
    auto a = row[0] * col[0];
    auto b = row[1] * col[4];
    auto c = row[2] * col[8];
    auto d = row[3] * col[12];
    auto value = a + b + c + d;
    return value;
}

inline std::array<float, 16> Mul(const float l[16], const float r[16])
{
    auto _11 = Dot(&l[0], &r[0]);
    auto _12 = Dot(&l[0], &r[1]);
    auto _13 = Dot(&l[0], &r[2]);
    auto _14 = Dot(&l[0], &r[3]);
    auto _21 = Dot(&l[4], &r[0]);
    auto _22 = Dot(&l[4], &r[1]);
    auto _23 = Dot(&l[4], &r[2]);
    auto _24 = Dot(&l[4], &r[3]);
    auto _31 = Dot(&l[8], &r[0]);
    auto _32 = Dot(&l[8], &r[1]);
    auto _33 = Dot(&l[8], &r[2]);
    auto _34 = Dot(&l[8], &r[3]);
    auto _41 = Dot(&l[12], &r[0]);
    auto _42 = Dot(&l[12], &r[1]);
    auto _43 = Dot(&l[12], &r[2]);
    auto _44 = Dot(&l[12], &r[3]);

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

inline std::array<float, 4> QuaternionAxisAngle(const std::array<float, 3> &axis, float angle)
{
    auto angle_half = angle / 2;
    auto sin = std::sin(angle_half);
    auto cos = std::cos(angle_half);
    return {axis[0] * sin, axis[1] * sin, axis[2] * sin, cos};
}

inline std::array<float, 4> QuaternionConjugate(const std::array<float, 4> &v)
{
    return {-v[0], -v[1], -v[2], v[3]};
}

inline std::array<float, 3> QuaternionXDir(const std::array<float, 4> &v)
{
    auto x = v[0];
    auto y = v[1];
    auto z = v[2];
    auto w = v[3];
    return {w * w + x * x - y * y - z * z, (x * y + z * w) * 2, (z * x - y * w) * 2};
}

inline std::array<float, 3> QuaternionYDir(const std::array<float, 4> &v)
{
    auto x = v[0];
    auto y = v[1];
    auto z = v[2];
    auto w = v[3];
    return {(x * y - z * w) * 2, w * w - x * x + y * y - z * z, (y * z + x * w) * 2};
}

inline std::array<float, 3> QuaternionZDir(const std::array<float, 4> &v)
{
    auto x = v[0];
    auto y = v[1];
    auto z = v[2];
    auto w = v[3];
    return {(z * x + y * w) * 2, (y * z - x * w) * 2, w * w - x * x - y * y + z * z};
}

inline std::array<float, 16> QuaternionMatrix(const std::array<float, 4> &q)
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

inline std::array<float, 3> QuaternionRotatePosition(const std::array<float, 4> &q, const std::array<float, 3> &v)
{
    auto x = QuaternionXDir(q);
    auto y = QuaternionYDir(q);
    auto z = QuaternionZDir(q);
    return x * v[0] + y * v[1] + z * v[2];
}

inline std::array<float, 4> operator*(const std::array<float, 4> &lhs, const std::array<float, 4> &rhs)
{
    float ax = lhs[0];
    float ay = lhs[1];
    float az = lhs[2];
    float aw = lhs[3];
    float bx = rhs[0];
    float by = rhs[1];
    float bz = rhs[2];
    float bw = rhs[3];
    return {
        ax * bw + aw * bx + ay * bz - az * by,
        ay * bw + aw * by + az * bx - ax * bz,
        az * bw + aw * bz + ax * by - ay * bx,
        aw * bw - ax * bx - ay * by - az * bz,
    };
}

struct Transform
{
    std::array<float, 3> position;
    std::array<float, 4> rotation;

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
        auto inv_t = QuaternionRotatePosition(inv_r, -position);
        return {inv_t, inv_r};
    }
};

} // namespace fpalg
