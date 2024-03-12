#include "vc/texturing/FlatteningAlgorithm.hpp"

#include "vc/core/util/FloatComparison.hpp"
#include "vc/core/util/MeshMath.hpp"

using namespace volcart;
using namespace volcart::texturing;

namespace mm = volcart::meshmath;

auto FlatteningAlgorithm::getUVMap() -> UVMap::Pointer
{
    // Setup uvMap
    auto uvMap = UVMap::New();

    // Get bounds
    auto bb = ITKMesh::BoundingBoxType::New();
    bb->SetPoints(output_->GetPoints());
    bb->ComputeBoundingBox();

    auto uMin = bb->GetBounds()[0];
    auto uMax = bb->GetBounds()[1];
    auto vMin = bb->GetBounds()[4];
    auto vMax = bb->GetBounds()[5];

    // Set the UV map ratio and get linear scale factor
    auto scale = sqrt(mm::SurfaceArea(mesh_) / mm::SurfaceArea(output_));
    auto aspectWidth = std::abs(uMax - uMin);
    auto aspectHeight = std::abs(vMax - vMin);
    uvMap->ratio(aspectWidth * scale, aspectHeight * scale);

    // Calculate uv coordinates
    cv::Vec2d uv;
    for (auto it = output_->GetPoints()->Begin();
         it != output_->GetPoints()->End(); ++it) {
        uv[0] = (it->Value()[0] - uMin) / (uMax - uMin);
        uv[1] = (it->Value()[2] - vMin) / (vMax - vMin);

        // Add the uv coordinates into our map at the point index specified
        uvMap->set(it->Index(), uv);
    }

    return uvMap;
}
