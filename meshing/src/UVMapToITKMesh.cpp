#include "vc/meshing/UVMapToITKMesh.hpp"

#include "vc/meshing/DeepCopy.hpp"

using namespace volcart;
using namespace volcart::meshing;

void UVMapToITKMesh::setMesh(const ITKMesh::Pointer& m) { inputMesh_ = m; }

void UVMapToITKMesh::setUVMap(UVMap u) { inputUVMap_ = std::move(u); }

void UVMapToITKMesh::setScaleToUVDimensions(bool b) { scaleMesh_ = b; }

ITKMesh::Pointer UVMapToITKMesh::compute()
{
    // Setup the output mesh with the faces of the input mesh
    outputMesh_ = ITKMesh::New();
    DeepCopy(inputMesh_, outputMesh_, false, true);

    // The vertex normal for UV faces is always the same
    ITKPixel n;
    n[0] = 0;
    n[1] = 1;
    n[2] = 0;

    // Copy the UV positions as the new point positions
    ITKPoint p;
    for (auto pt = inputMesh_->GetPoints()->Begin();
         pt != inputMesh_->GetPoints()->End(); ++pt) {
        auto uv = inputUVMap_.get(pt.Index());

        // If enabled, do position scaling
        if (scaleMesh_) {
            uv[0] *= inputUVMap_.ratio().width - 1;
            uv[1] *= inputUVMap_.ratio().height - 1;
        }

        // Assign to the point
        p[0] = uv[0];
        p[1] = 0;
        p[2] = uv[1];

        // Assign to the mesh
        outputMesh_->SetPoint(pt.Index(), p);
        outputMesh_->SetPointData(pt.Index(), n);
    }

    return outputMesh_;
}
ITKMesh::Pointer UVMapToITKMesh::getUVMesh() const { return outputMesh_; }