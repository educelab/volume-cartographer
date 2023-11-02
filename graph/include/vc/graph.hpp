#pragma once

/** @file */

#include <opencv2/core.hpp>

#include "vc/graph/core.hpp"
#include "vc/graph/meshing.hpp"
#include "vc/graph/texturing.hpp"

namespace volcart
{

/** @brief Register all VC provided nodes with the Smeagol library */
void RegisterNodes();

}  // namespace volcart