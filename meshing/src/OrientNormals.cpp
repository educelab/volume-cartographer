#include "vc/meshing/OrientNormals.hpp"

#include <opencv2/core.hpp>

#include "vc/meshing/DeepCopy.hpp"

using namespace volcart;
using namespace volcart::meshing;

namespace
{
auto ComputeMeshCentroid(const ITKMesh::Pointer& mesh) -> cv::Vec3d
{
    cv::Vec3d centroid{0, 0, 0};
    double cnt = 1.;
    for (auto it = mesh->GetPoints()->Begin(); it != mesh->GetPoints()->End();
         ++it) {
        auto vert = it.Value();
        const cv::Vec3d point{vert[0], vert[1], vert[2]};
        centroid = centroid + (point - centroid) / cnt;
        cnt += 1.;
    }
    return centroid;
}
}  // namespace

void OrientNormals::setMesh(const ITKMesh::Pointer& mesh) { input_ = mesh; }

auto OrientNormals::getMesh() const -> ITKMesh::Pointer { return output_; }

auto OrientNormals::compute() -> ITKMesh::Pointer
{
    // Compute the mesh refPt
    auto refPt = ::ComputeMeshCentroid(input_);

    // Get a count of the vectors which points towards and away from the
    // reference point
    std::size_t coDir{0};
    std::size_t antiDir{0};

    for (auto it = input_->GetPoints()->Begin();
         it != input_->GetPoints()->End(); ++it) {
        // Temp variables
        ITKPoint vert;
        ITKPixel normal;

        // Convert point data to cv::Vec3d
        vert = it->Value();
        input_->GetPointData(it.Index(), &normal);
        cv::Vec3d pt{vert[0], vert[1], vert[2]};
        cv::Vec3d n{normal[0], normal[1], normal[2]};
        n = cv::normalize(n);

        // Accumulate the relative direction of the normals for this face.
        if (n.dot(cv::normalize(refPt - pt)) > 0) {
            coDir++;
        } else {
            antiDir++;
        }
    }

    // If more face normals were facing outwards than inwards, they are flipped.
    auto flip = antiDir > coDir;

    // Setup output mesh
    output_ = ITKMesh::New();
    DeepCopy(input_, output_);

    // Flip the normals as required
    if (flip) {
        for (auto it = input_->GetPoints()->Begin();
             it != input_->GetPoints()->End(); ++it) {
            // Convert point data to cv::Vec3d
            ITKPixel normal;
            input_->GetPointData(it.Index(), &normal);
            normal *= -1;
            input_->SetPointData(it.Index(), normal);
        }
    }

    return output_;
}
