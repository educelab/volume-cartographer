/** @file itk2bullet.cpp */

#include "meshing/itk2bullet.h"

namespace volcart
{
namespace meshing
{

itk2bullet::itk2bullet(
    ITKMesh::Pointer input, btSoftBodyWorldInfo& worldInfo, btSoftBody** output)
{

    ITKPoint old_point;
    btVector3* new_point;

    for (ITKPointIterator it = input->GetPoints()->Begin();
         it != input->GetPoints()->End(); ++it) {
        // copy vertex info from itk mesh to btSoftBody
        old_point = it->Value();
        new_point = new btVector3(
            static_cast<float>(old_point[0]), static_cast<float>(old_point[1]),
            static_cast<float>(old_point[2]));

        if (it == input->GetPoints()->Begin()) {
            *output = new btSoftBody(&worldInfo, 1, new_point, 0);
        } else {
            (*output)->appendNode(*new_point, 0);
        }
    }

    // convert the cells to faces
    unsigned long v0, v1, v2;
    v0 = v1 = v2 = 0;

    for (ITKCellIterator cell = input->GetCells()->Begin();
         cell != input->GetCells()->End(); ++cell) {

        v0 = cell.Value()->GetPointIds()[0];
        v1 = cell.Value()->GetPointIds()[1];
        v2 = cell.Value()->GetPointIds()[2];

        (*output)->appendLink(v0, v1);
        (*output)->appendLink(v1, v2);
        (*output)->appendLink(v2, v0);

        (*output)->appendFace(v0, v1, v2);
    }
};

bullet2itk::bullet2itk(btSoftBody* input, ITKMesh::Pointer output)
{
    ITKCell::CellAutoPointer cellpointer;
    ITKPoint p;
    ITKPixel n;
    int NUM_OF_POINTS = input->m_nodes.size();

    // iterate through points of bullet mesh (softBody)
    for (int i = 0; i < NUM_OF_POINTS; ++i) {

        p[0] = input->m_nodes[i].m_x.x();
        p[1] = input->m_nodes[i].m_x.y();
        p[2] = input->m_nodes[i].m_x.z();

        n[0] = input->m_nodes[i].m_n.x();
        n[1] = input->m_nodes[i].m_n.y();
        n[2] = input->m_nodes[i].m_n.z();

        output->SetPoint(i, p);
        output->SetPointData(i, n);
    }
};

}  // namespace meshing
}  //  namespace volcart
