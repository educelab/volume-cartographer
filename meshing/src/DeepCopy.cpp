/** @file DeepCopy.cpp */

#include "vc/meshing/DeepCopy.hpp"

namespace volcart::meshing
{

void DeepCopy(
    const ITKMesh::Pointer& input,
    const ITKMesh::Pointer& output,
    bool copyVertices,
    bool copyFaces)
{
    // Copy the points and their normals
    if (copyVertices) {
        ITKPoint p;
        ITKPixel n;
        for (auto pt = input->GetPoints()->Begin();
             pt != input->GetPoints()->End(); ++pt) {
            p = pt->Value();
            input->GetPointData(pt.Index(), &n);

            output->SetPoint(pt.Index(), p);
            output->SetPointData(pt.Index(), n);
        }
    }

    // Copy the faces
    if (copyFaces) {
        ITKCell::CellAutoPointer c;
        for (auto cell = input->GetCells()->Begin();
             cell != input->GetCells()->End(); ++cell) {

            c.TakeOwnership(new ITKTriangle);
            for (uint32_t pointId = 0;
                 pointId < cell.Value()->GetNumberOfPoints(); ++pointId) {
                c->SetPointId(
                    pointId, cell.Value()->GetPointIdsContainer()[pointId]);
            }

            output->SetCell(cell->Index(), c);
        }
    }
}
}  // namespace volcart::meshing
