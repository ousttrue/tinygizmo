#include "trs.h"
#include "rigid_transform.h"
#include "minalg.h"
#include "../screenstate/castalg.h"

namespace tinygizmo
{

std::array<float, 16> TRS::matrix() const
{
    return castalg::ref_cast<std::array<float, 16>>(castalg::ref_cast<rigid_transform>(*this).matrix());
}

} // namespace tinygizmo
