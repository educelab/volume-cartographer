/** @file OrderedPointSetMesher.cpp */

#include "vc/meshing/OrderedPointSetMesher.hpp"
#include <iostream>
#include "vc/meshing/CalculateNormals.hpp"

using namespace volcart::meshing;

volcart::ITKMesh::Pointer OrderedPointSetMesher::compute()
{
    // Verify before computation
    if (input_.empty()) {
        throw std::invalid_argument("Attempted to mesh empty point set.");
    }

    // Create a clean output mesh
    output_ = ITKMesh::New();

    // Transfer the vertex info
    ITKPoint tmpPt;
    size_t cnt = 0;
    for (auto& i : input_) {
        tmpPt[0] = i[0];
        tmpPt[1] = i[1];
        tmpPt[2] = i[2];

        output_->SetPoint(cnt, tmpPt);
        ++cnt;
    }

    // Return early if we're not triangulating
    if (!generateTriangles_) {
        return output_;
    }

    // Creates 2 cells per iteration and adds them to the mesh
    size_t p0, p1, p2, p3;
    for (size_t i = 0; i < input_.height() - 1; i++) {
        for (size_t j = 0; j < input_.width() - 1; j++) {

            // Get the indices for the faces adjacent along the "hypotenuse"
            p0 = i * input_.width() + j;
            p1 = p0 + 1;
            p2 = p1 + input_.width();
            p3 = p2 - 1;

            if (output_->GetPoint(p0)[2] == -1 ||
                output_->GetPoint(p1)[2] == -1 ||
                output_->GetPoint(p2)[2] == -1 ||
                output_->GetPoint(p3)[2] == -1) {
                continue;
            }
            if (p0 >= output_->GetNumberOfPoints() ||
                p1 >= output_->GetNumberOfPoints() ||
                p2 >= output_->GetNumberOfPoints() ||
                p3 >= output_->GetNumberOfPoints()) {
                throw std::out_of_range(
                    "Predicted vertex index for face generation out of range "
                    "of point set.");
            }

            add_cell_(p1, p2, p3);
            add_cell_(p0, p1, p3);
        }
    }

    // Sets the normals for the points and faces
    volcart::meshing::CalculateNormals calcNorm(output_);
    calcNorm.compute();
    output_ = calcNorm.getMesh();

    return output_;
}

void OrderedPointSetMesher::add_cell_(size_t a, size_t b, size_t c)
{
    ITKCell::CellAutoPointer currentC;

    currentC.TakeOwnership(new ITKTriangle);

    currentC->SetPointId(0, a);
    currentC->SetPointId(1, b);
    currentC->SetPointId(2, c);

    output_->SetCell(output_->GetNumberOfCells(), currentC);
}
