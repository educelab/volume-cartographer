#include "vc/meshing/OrientNormals.hpp"

#include "vc/core/util/Iteration.hpp"
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

void OrientNormals::setReferenceMode(OrientNormals::ReferenceMode mode)
{
    mode_ = mode;
}

auto OrientNormals::referenceMode() const -> OrientNormals::ReferenceMode
{
    return mode_;
}

void OrientNormals::setReferencePoint(const cv::Vec3d& point)
{
    mode_ = ReferenceMode::Manual;
    refPt_ = point;
}

auto OrientNormals::referencePoint() const -> cv::Vec3d { return refPt_; }

auto OrientNormals::compute() -> ITKMesh::Pointer
{
    // Get the reference point based on the reference mode
    cv::Vec3d refPt;
    if (mode_ == ReferenceMode::Centroid) {
        refPt = ::ComputeMeshCentroid(input_);
    } else if (mode_ == ReferenceMode::Manual) {
        refPt = refPt_;
    } else {
        refPt = ::ComputeMeshCentroid(input_);
    }

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
        for (auto it = output_->GetPoints()->Begin();
             it != output_->GetPoints()->End(); ++it) {
            // Convert point data to cv::Vec3d
            ITKPixel normal;
            output_->GetPointData(it.Index(), &normal);
            normal *= -1;
            output_->SetPointData(it.Index(), normal);
        }

        for (auto it = output_->GetCells()->Begin();
             it != output_->GetCells()->End(); ++it) {
            auto pointIds = it->Value()->GetPointIdsContainer().flip();
            for (auto [localId, id] : enumerate(pointIds)) {
                it->Value()->SetPointId(static_cast<int>(localId), id);
            }
        }
    }

    return output_;
}
