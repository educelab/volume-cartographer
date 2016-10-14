//
// Created by Hannah Hatch on 8/23/16.
//

#include "meshing/OrderedPointSetMesher.h"
#include <iostream>
#include "meshing/CalculateNormals.h"

using namespace volcart::meshing;

void OrderedPointSetMesher::compute()
{
    // Verify before computation
    if (input_.empty())
        throw std::invalid_argument("Attempted to mesh empty point set.");

    // Create a clean output mesh
    output_ = ITKMesh::New();

    // Transfer the vertex info
    ITKPoint temp_pt;
    size_t cnt = 0;
    for (auto& i : input_) {
        temp_pt[0] = i[0];
        temp_pt[1] = i[1];
        temp_pt[2] = i[2];

        output_->SetPoint(cnt, temp_pt);
        ++cnt;
    }

    // Creates 2 cells per iteration and adds them to the mesh
    size_t p0, p1, p2, p3;
    for (auto i = 0; i < input_.height() - 1; i++) {
        for (auto j = 0; j < input_.width() - 1; j++) {

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

            addCell_(p1, p2, p3);
            addCell_(p0, p1, p3);
        }  // j loop
    }      // i loop

    // Sets the normals for the points and faces
    volcart::meshing::CalculateNormals calcNorm(output_);
    calcNorm.compute();
    output_ = calcNorm.getMesh();
}

void OrderedPointSetMesher::addCell_(size_t a, size_t b, size_t c)
{
    ITKCell::CellAutoPointer current_C;

    current_C.TakeOwnership(new ITKTriangle);

    current_C->SetPointId(0, a);
    current_C->SetPointId(1, b);
    current_C->SetPointId(2, c);

    output_->SetCell(output_->GetNumberOfCells(), current_C);
}
