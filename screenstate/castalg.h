#pragma once
#include <iosfwd>
#include <math.h>

namespace castalg
{

template <typename T, typename S>
inline const T &ref_cast(const S &s)
{
    static_assert(sizeof(S) == sizeof(T), "must same size");
    return *((T *)&s);
}

template <typename T, typename S>
inline T &ref_cast(S &s)
{
    static_assert(sizeof(S) == sizeof(T), "must same size");
    return *((T *)&s);
}

/////////////////////////////////////////////////////////////////////////////
template <typename T>
struct T3
{
    T x;
    T y;
    T z;

    T3()
    {
    }

    T3(const T &_x, const T &_y, const T &_z)
        : x(_x), y(_y), z(_z)
    {
    }

    template <typename V>
    T3(const V &value)
    {
        static_assert(sizeof(V) == sizeof(T) * 3, "must size == T * 3");
        *this = *(T3<T> *)&value;
    }

    template <typename V>
    V &ref()
    {
        static_assert(sizeof(V) == sizeof(T) * 3, "must size == T * 3");
        return *(V *)this;
    }

    template <typename V>
    const V &ref() const
    {
        static_assert(sizeof(V) == sizeof(T) * 3, "must size == T * 3");
        return *(V *)this;
    }

    T3 operator-() const
    {
        return T3(-x, -y, -z);
    }

    T3 operator+(const T3 &rhs) const
    {
        return T3(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    T3 operator*(const T &scalar) const
    {
        return T3(x * scalar, y * scalar, z * scalar);
    }
};
using float3 = T3<float>;

template <typename T, typename V>
T3<T> &ref_t3(V &value)
{
    static_assert(sizeof(V) == sizeof(T) * 3, "must size == T * 3");
    return *(T3<T> *)&value;
}

template <typename V>
T3<float> &ref_float3(V &value)
{
    return ref_t3<float, V>(value);
}

template <typename T, typename V>
const T3<T> &ref_t3(const V &value)
{
    static_assert(sizeof(V) == sizeof(T) * 3, "must size == T * 3");
    return *(T3<T> *)&value;
}

template <typename V>
const T3<float> &ref_float3(const V &value)
{
    return ref_t3<float, V>(value);
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const T3<T> &rhs)
{
    os << "[" << rhs.x << ',' << rhs.y << ',' << rhs.z << ']';
    return os;
}

/////////////////////////////////////////////////////////////////////////////
template <typename T>
struct T4
{
    T x;
    T y;
    T z;
    T w;

    T4()
    {
    }

    T4(T _x, T _y, T _z, T _w)
        : x(_x), y(_y), z(_z), w(_w)
    {
    }
};
using float4 = T4<float>;

template <typename T, typename V>
const T4<T> &ref_t4(const V &value)
{
    static_assert(sizeof(V) == sizeof(T) * 4, "must size == T * 4");
    return *(T4<T> *)&value;
}

/////////////////////////////////////////////////////////////////////////////
struct quaternion
{
    float4 values;

    quaternion(float x, float y, float z, float w)
    {
        values = float4{x, y, z, w};
    }

    static quaternion axisAngle(const float3 &axis, float angle)
    {
        auto angle_half = angle / 2;
        auto sin = std::sin(angle_half);
        auto cos = std::cos(angle_half);
        return quaternion(axis.x * sin, axis.y * sin, axis.z * sin, cos);
    }

    quaternion operator*(const quaternion &rhs) const
    {
        auto &a = values;
        auto &b = rhs.values;
        float4 r{
            a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y,
            a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
            a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        };
        return quaternion(r.x, r.y, r.z, r.w);
    }

    quaternion conjugate() const
    {
        return quaternion(-values.x,
                          -values.y,
                          -values.z,
                          values.w);
    }

    float3 xDir() const
    {
        auto &v = values;
        return {v.w * v.w + v.x * v.x - v.y * v.y - v.z * v.z, (v.x * v.y + v.z * v.w) * 2, (v.z * v.x - v.y * v.w) * 2};
    }
    float3 yDir() const
    {
        auto &v = values;
        return {(v.x * v.y - v.z * v.w) * 2, v.w * v.w - v.x * v.x + v.y * v.y - v.z * v.z, (v.y * v.z + v.x * v.w) * 2};
    }
    float3 zDir() const
    {
        auto &v = values;
        return {(v.z * v.x + v.y * v.w) * 2, (v.y * v.z - v.x * v.w) * 2, v.w * v.w - v.x * v.x - v.y * v.y + v.z * v.z};
    }
    float3 rotate(const float3 &v) const
    {
        return xDir() * v.x + yDir() * v.y + zDir() * v.z;
    }
};

template <typename V>
quaternion &ref_quaternion(V &value)
{
    return ref_cast<quaternion>(value);
}
template <typename V>
const quaternion &ref_quaternion(const V &value)
{
    return ref_cast<const quaternion>(value);
}

/////////////////////////////////////////////////////////////////////////////
struct matrix
{
    float _00, _01, _02, _03;
    float _10, _11, _12, _13;
    float _20, _21, _22, _23;
    float _30, _31, _32, _33;

    float operator[](int index) const
    {
        switch (index)
        {
        case 0:
            return _00;
        case 1:
            return _01;
        case 2:
            return _02;
        case 3:
            return _03;
        case 4:
            return _10;
        case 5:
            return _11;
        case 6:
            return _12;
        case 7:
            return _13;
        case 8:
            return _20;
        case 9:
            return _21;
        case 10:
            return _22;
        case 11:
            return _23;
        case 12:
            return _30;
        case 13:
            return _31;
        case 14:
            return _32;
        case 15:
            return _33;

        default:
            throw;
        }
    }

    matrix operator*(const matrix &rhs)
    {
        auto &lhs = *this;
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

    static matrix translation(const float3 &t)
    {
        return {
            1, 0, 0, 0,       //
            0, 1, 0, 0,       //
            0, 0, 1, 0,       //
            t.x, t.y, t.z, 1, //
        };
    }

    static matrix rotation(const quaternion &q)
    {
        auto x = q.xDir();
        auto y = q.yDir();
        auto z = q.zDir();
        return {
            x.x, x.y, x.z, 0,
            y.x, y.y, y.z, 0,
            z.x, z.y, z.z, 0,
            0, 0, 0, 1};
    }

    static matrix perspectiveGLRH(float fovy, float aspect, float zNear, float zFar)
    {
        auto f = static_cast<float>(1.0f / tan(fovy / 2.0));

        return {
            (f / aspect),
            0.0f,
            0.0f,
            0.0f, //
            0.0f,
            f,
            0.0f,
            0.0f, //
            0.0,
            0.0,
            (zFar + zNear) / (zNear - zFar),
            -1.0, //
            0.0,
            0.0,
            (2 * zFar * zNear) / (zNear - zFar),
            0.0,
        };
    }
};

struct transform
{
    float3 position;
    quaternion rotation;

    matrix matrix() const
    {
        auto r = castalg::matrix::rotation(rotation);
        r._30 = position.x;
        r._31 = position.y;
        r._32 = position.z;
        return r;
    }

    transform inverse() const
    {
        auto inv_r = rotation.conjugate();
        return {inv_r.rotate(-position), inv_r};
    }
};

} // namespace castalg
