#include "vc/core/util/MeshMath.hpp"

#include "vc/core/util/Logging.hpp"
namespace volcart
{
namespace meshmath
{

double SurfaceArea(const ITKMesh::Pointer& input)
{
    double surfaceArea = 0;

    uint64_t vID0 = 0, vID1 = 0, vID2 = 0;
    double a = 0, b = 0, c = 0, p = 0;
    for (auto cell = input->GetCells()->Begin();
         cell != input->GetCells()->End(); ++cell) {
        vID0 = cell.Value()->GetPointIds()[0];
        vID1 = cell.Value()->GetPointIds()[1];
        vID2 = cell.Value()->GetPointIds()[2];

        // Get the side lengths
        a = input->GetPoint(vID0).EuclideanDistanceTo(input->GetPoint(vID1));
        b = input->GetPoint(vID0).EuclideanDistanceTo(input->GetPoint(vID2));
        c = input->GetPoint(vID1).EuclideanDistanceTo(input->GetPoint(vID2));

        // Sort the side lengths so that a >= b >= c
        double na, nb, nc;
        nc = std::min(a, std::min(b, c));
        na = std::max(a, std::max(b, c));
        nb = a + b + c - na - nc;

        // Calculate the area
        p = (na + (nb + nc)) * (nc - (na - nb)) * (nc + (na - nb)) *
            (na + (nb - nc));
        double sa = 0.25 * sqrt(p);

        // Can get NaN's when using standard C++ math. Explore something like
        // GMP
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
}  // namespace meshmath
}  // namespace volcart
