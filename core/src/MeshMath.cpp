#include "vc/core/util/MeshMath.hpp"

#include "vc/core/util/Logging.hpp"

namespace volcart::meshmath
{

double SurfaceArea(const ITKMesh::Pointer& mesh)
{
    if(mesh == nullptr) {
        throw std::runtime_error("Failed to calculate surface area. Mesh is nullptr.");
    }
    double surfaceArea{0};
    for (auto cell = mesh->GetCells()->Begin(); cell != mesh->GetCells()->End();
         ++cell) {

        // Get vertices
        auto vID0 = cell->Value()->GetPointIds()[0];
        auto vID1 = cell->Value()->GetPointIds()[1];
        auto vID2 = cell->Value()->GetPointIds()[2];

        // Get the side lengths
        auto a = mesh->GetPoint(vID0).EuclideanDistanceTo(mesh->GetPoint(vID1));
        auto b = mesh->GetPoint(vID0).EuclideanDistanceTo(mesh->GetPoint(vID2));
        auto c = mesh->GetPoint(vID1).EuclideanDistanceTo(mesh->GetPoint(vID2));

        // Get cell surface area
        auto sa = TriangleArea(a, b, c);

        // Note: Can get NaN's when using std::math
        if (std::isnan(sa)) {
            Logger()->error(
                "volcart::meshMath: Warning: NaN surface area for face[{}]. "
                "Evaluating as 0.",
                cell.Index());
            sa = 0.0;
        }
        surfaceArea += sa;
    }

    return surfaceArea;
}
}  // namespace volcart::meshmath
