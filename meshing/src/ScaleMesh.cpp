//
// Created by Seth Parker on 10/22/15.
//

/**@file ScaleMesh.cpp  */

#include "meshing/ScaleMesh.hpp"

namespace volcart
{
namespace meshing
{

void ScaleMesh(
    const ITKMesh::Pointer& input,
    const ITKMesh::Pointer& output,
    double scaleFactor)
{
    // Copy the points and their normals
    ITKPoint p;
    ITKPixel n;
    for (auto pt = input->GetPoints()->Begin(); pt != input->GetPoints()->End();
         ++pt) {
        p = pt->Value();
        input->GetPointData(pt.Index(), &n);
        p[0] *= scaleFactor;
        p[1] *= scaleFactor;
        p[2] *= scaleFactor;
        output->SetPoint(pt.Index(), p);
        output->SetPointData(pt.Index(), n);
    }
    // Copy the faces
    ITKCell::CellAutoPointer c;
    for (auto cell = input->GetCells()->Begin();
         cell != input->GetCells()->End(); ++cell) {
        c.TakeOwnership(new ITKTriangle);
        for (uint32_t pointId = 0; pointId < cell.Value()->GetNumberOfPoints();
             ++pointId) {
            c->SetPointId(
                    pointId, cell.Value()->GetPointIdsContainer()[pointId]);
        }
        output->SetCell(cell->Index(), c);
    }
};
}
}
