/** @file DeepCopy.cpp */

#include "meshing/DeepCopy.h"

namespace volcart
{
namespace meshing
{
DeepCopy::DeepCopy(ITKMesh::Pointer input, ITKMesh::Pointer output)
{

    // Copy the points and their normals
    ITKPoint p;
    ITKPixel n;
    for (ITKPointIterator pt = input->GetPoints()->Begin();
         pt != input->GetPoints()->End(); ++pt) {
        p = pt->Value();
        input->GetPointData(pt.Index(), &n);

        output->SetPoint(pt.Index(), p);
        output->SetPointData(pt.Index(), n);
    }

    // Copy the faces
    ITKCell::CellAutoPointer c;
    for (ITKCellIterator cell = input->GetCells()->Begin();
         cell != input->GetCells()->End(); ++cell) {

        c.TakeOwnership(new ITKTriangle);
        for (uint32_t p_id = 0; p_id < cell.Value()->GetNumberOfPoints();
             ++p_id)
            c->SetPointId(p_id, cell.Value()->GetPointIdsContainer()[p_id]);

        output->SetCell(cell->Index(), c);
    }
}  // DeepCopy
}  // meshing
}  // volcart
