///////////////////////
//   Utility Math    //
///////////////////////

static const minalg::float4x4 Identity4x4 = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
static const minalg::float3x3 Identity3x3 = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
static const float tau = 6.28318530718f;

void flush_to_zero(minalg::float3 &f)
{
    if (std::abs(f.x) < 0.02f)
        f.x = 0.f;
    if (std::abs(f.y) < 0.02f)
        f.y = 0.f;
    if (std::abs(f.z) < 0.02f)
        f.z = 0.f;
}

// 32 bit Fowler鋒oll坊o Hash
uint32_t hash_fnv1a(const std::string &str)
{
    static const uint32_t fnv1aBase32 = 0x811C9DC5u;
    static const uint32_t fnv1aPrime32 = 0x01000193u;

    uint32_t result = fnv1aBase32;

    for (auto &c : str)
    {
        result ^= static_cast<uint32_t>(c);
        result *= fnv1aPrime32;
    }
    return result;
}

minalg::float3 snap(const minalg::float3 &value, const float snap)
{
    if (snap > 0.0f)
        return minalg::float3(floor(value / snap) * snap);
    return value;
}

minalg::float4 make_rotation_quat_axis_angle(const minalg::float3 &axis, float angle)
{
    return {axis * std::sin(angle / 2), std::cos(angle / 2)};
}

minalg::float4 make_rotation_quat_between_vectors_snapped(const minalg::float3 &from, const minalg::float3 &to, const float angle)
{
    auto a = normalize(from);
    auto b = normalize(to);
    auto snappedAcos = std::floor(std::acos(dot(a, b)) / angle) * angle;
    return make_rotation_quat_axis_angle(normalize(cross(a, b)), snappedAcos);
}


template <typename T>
T clamp(const T &val, const T &min, const T &max) { return std::min(std::max(val, min), max); }

struct ray
{
    minalg::float3 origin, direction;
};

ray transform(const rigid_transform &p, const ray &r) { return {p.transform_point(r.origin), p.transform_vector(r.direction)}; }
ray detransform(const rigid_transform &p, const ray &r) { return {p.detransform_point(r.origin), p.detransform_vector(r.direction)}; }

minalg::float3 transform_coord(const minalg::float4x4 &transform, const minalg::float3 &coord)
{
    auto r = mul(transform, minalg::float4(coord, 1));
    return (r.xyz() / r.w);
}
minalg::float3 transform_vector(const minalg::float4x4 &transform, const minalg::float3 &vector) { return mul(transform, minalg::float4(vector, 0)).xyz(); }
void transform(const float scale, ray &r)
{
    r.origin *= scale;
    r.direction *= scale;
}
void detransform(const float scale, ray &r)
{
    r.origin /= scale;
    r.direction /= scale;
}

/////////////////////////////////////////
// Ray-Geometry Intersection Functions //
/////////////////////////////////////////

bool intersect_ray_plane(const ray &ray, const minalg::float4 &plane, float *hit_t)
{
    float denom = dot(plane.xyz(), ray.direction);
    if (std::abs(denom) == 0)
        return false;
    if (hit_t)
        *hit_t = -dot(plane, minalg::float4(ray.origin, 1)) / denom;
    return true;
}

bool intersect_ray_triangle(const ray &ray, const minalg::float3 &v0, const minalg::float3 &v1, const minalg::float3 &v2, float *hit_t)
{
    auto e1 = v1 - v0, e2 = v2 - v0, h = cross(ray.direction, e2);
    auto a = dot(e1, h);
    if (std::abs(a) == 0)
        return false;

    float f = 1 / a;
    auto s = ray.origin - v0;
    auto u = f * dot(s, h);
    if (u < 0 || u > 1)
        return false;

    auto q = cross(s, e1);
    auto v = f * dot(ray.direction, q);
    if (v < 0 || u + v > 1)
        return false;

    auto t = f * dot(e2, q);
    if (t < 0)
        return false;

    if (hit_t)
        *hit_t = t;
    return true;
}

bool intersect_ray_mesh(const ray &ray, const geometry_mesh &mesh, float *hit_t)
{
    float best_t = std::numeric_limits<float>::infinity(), t;
    int32_t best_tri = -1;
    for (auto &tri : mesh.triangles)
    {
        if (intersect_ray_triangle(ray, mesh.vertices[tri[0]].position, mesh.vertices[tri[1]].position, mesh.vertices[tri[2]].position, &t) && t < best_t)
        {
            best_t = t;
            best_tri = uint32_t(&tri - mesh.triangles.data());
        }
    }
    if (best_tri == -1)
        return false;
    if (hit_t)
        *hit_t = best_t;
    return true;
}

///////////////////////////////
// Geometry + Mesh Utilities //
///////////////////////////////

void compute_normals(geometry_mesh &mesh)
{
    static const double NORMAL_EPSILON = 0.0001;

    std::vector<uint32_t> uniqueVertIndices(mesh.vertices.size(), 0);
    for (uint32_t i = 0; i < uniqueVertIndices.size(); ++i)
    {
        if (uniqueVertIndices[i] == 0)
        {
            uniqueVertIndices[i] = i + 1;
            const minalg::float3 v0 = mesh.vertices[i].position;
            for (auto j = i + 1; j < mesh.vertices.size(); ++j)
            {
                const minalg::float3 v1 = mesh.vertices[j].position;
                if (length2(v1 - v0) < NORMAL_EPSILON)
                {
                    uniqueVertIndices[j] = uniqueVertIndices[i];
                }
            }
        }
    }

    uint32_t idx0, idx1, idx2;
    for (auto &t : mesh.triangles)
    {
        idx0 = uniqueVertIndices[t.x] - 1;
        idx1 = uniqueVertIndices[t.y] - 1;
        idx2 = uniqueVertIndices[t.z] - 1;

        geometry_vertex &v0 = mesh.vertices[idx0], &v1 = mesh.vertices[idx1], &v2 = mesh.vertices[idx2];
        const minalg::float3 n = cross(v1.position - v0.position, v2.position - v0.position);
        v0.normal += n;
        v1.normal += n;
        v2.normal += n;
    }

    for (uint32_t i = 0; i < mesh.vertices.size(); ++i)
        mesh.vertices[i].normal = mesh.vertices[uniqueVertIndices[i] - 1].normal;
    for (geometry_vertex &v : mesh.vertices)
        v.normal = normalize(v.normal);
}

geometry_mesh make_box_geometry(const minalg::float3 &min_bounds, const minalg::float3 &max_bounds)
{
    const auto a = min_bounds, b = max_bounds;
    geometry_mesh mesh;
    mesh.vertices = {
        {{a.x, a.y, a.z}, {-1, 0, 0}},
        {{a.x, a.y, b.z}, {-1, 0, 0}},
        {{a.x, b.y, b.z}, {-1, 0, 0}},
        {{a.x, b.y, a.z}, {-1, 0, 0}},
        {{b.x, a.y, a.z}, {+1, 0, 0}},
        {{b.x, b.y, a.z}, {+1, 0, 0}},
        {{b.x, b.y, b.z}, {+1, 0, 0}},
        {{b.x, a.y, b.z}, {+1, 0, 0}},
        {{a.x, a.y, a.z}, {0, -1, 0}},
        {{b.x, a.y, a.z}, {0, -1, 0}},
        {{b.x, a.y, b.z}, {0, -1, 0}},
        {{a.x, a.y, b.z}, {0, -1, 0}},
        {{a.x, b.y, a.z}, {0, +1, 0}},
        {{a.x, b.y, b.z}, {0, +1, 0}},
        {{b.x, b.y, b.z}, {0, +1, 0}},
        {{b.x, b.y, a.z}, {0, +1, 0}},
        {{a.x, a.y, a.z}, {0, 0, -1}},
        {{a.x, b.y, a.z}, {0, 0, -1}},
        {{b.x, b.y, a.z}, {0, 0, -1}},
        {{b.x, a.y, a.z}, {0, 0, -1}},
        {{a.x, a.y, b.z}, {0, 0, +1}},
        {{b.x, a.y, b.z}, {0, 0, +1}},
        {{b.x, b.y, b.z}, {0, 0, +1}},
        {{a.x, b.y, b.z}, {0, 0, +1}},
    };
    mesh.triangles = {{0, 1, 2}, {0, 2, 3}, {4, 5, 6}, {4, 6, 7}, {8, 9, 10}, {8, 10, 11}, {12, 13, 14}, {12, 14, 15}, {16, 17, 18}, {16, 18, 19}, {20, 21, 22}, {20, 22, 23}};
    return mesh;
}

geometry_mesh make_cylinder_geometry(const minalg::float3 &axis, const minalg::float3 &arm1, const minalg::float3 &arm2, uint32_t slices)
{
    // Generated curved surface
    geometry_mesh mesh;

    for (uint32_t i = 0; i <= slices; ++i)
    {
        const float tex_s = static_cast<float>(i) / slices, angle = (float)(i % slices) * tau / slices;
        const minalg::float3 arm = arm1 * std::cos(angle) + arm2 * std::sin(angle);
        mesh.vertices.push_back({arm, normalize(arm)});
        mesh.vertices.push_back({arm + axis, normalize(arm)});
    }
    for (uint32_t i = 0; i < slices; ++i)
    {
        mesh.triangles.push_back({i * 2, i * 2 + 2, i * 2 + 3});
        mesh.triangles.push_back({i * 2, i * 2 + 3, i * 2 + 1});
    }

    // Generate caps
    uint32_t base = (uint32_t)mesh.vertices.size();
    for (uint32_t i = 0; i < slices; ++i)
    {
        const float angle = static_cast<float>(i % slices) * tau / slices, c = std::cos(angle), s = std::sin(angle);
        const minalg::float3 arm = arm1 * c + arm2 * s;
        mesh.vertices.push_back({arm + axis, normalize(axis)});
        mesh.vertices.push_back({arm, -normalize(axis)});
    }
    for (uint32_t i = 2; i < slices; ++i)
    {
        mesh.triangles.push_back({base, base + i * 2 - 2, base + i * 2});
        mesh.triangles.push_back({base + 1, base + i * 2 + 1, base + i * 2 - 1});
    }
    return mesh;
}

geometry_mesh make_lathed_geometry(const minalg::float3 &axis, const minalg::float3 &arm1, const minalg::float3 &arm2, int slices, const std::vector<minalg::float2> &points, const float eps = 0.0f)
{
    geometry_mesh mesh;
    for (int i = 0; i <= slices; ++i)
    {
        const float angle = (static_cast<float>(i % slices) * tau / slices) + (tau / 8.f), c = std::cos(angle), s = std::sin(angle);
        const minalg::float3x2 mat = {axis, arm1 * c + arm2 * s};
        for (auto &p : points)
            mesh.vertices.push_back({mul(mat, p) + eps, minalg::float3(0.f)});

        if (i > 0)
        {
            for (uint32_t j = 1; j < (uint32_t)points.size(); ++j)
            {
                uint32_t i0 = (i - 1) * uint32_t(points.size()) + (j - 1);
                uint32_t i1 = (i - 0) * uint32_t(points.size()) + (j - 1);
                uint32_t i2 = (i - 0) * uint32_t(points.size()) + (j - 0);
                uint32_t i3 = (i - 1) * uint32_t(points.size()) + (j - 0);
                mesh.triangles.push_back({i0, i1, i2});
                mesh.triangles.push_back({i0, i2, i3});
            }
        }
    }
    compute_normals(mesh);
    return mesh;
}
