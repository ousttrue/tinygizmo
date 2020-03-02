#pragma once
#include <array>

namespace tinygizmo
{

struct TRS
{
    std::array<float, 3> translation{};
    std::array<float, 4> rotation{0, 0, 0, 1};
    std::array<float, 3> scale{1, 1, 1};

    std::array<float, 16> matrix() const;
};
static_assert(sizeof(TRS) == 40, "TRS");

} // namespace tinygizmo
