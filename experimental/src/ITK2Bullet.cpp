/** @file ITK2Bullet.cpp */

#include "vc/experimental/meshing/ITK2Bullet.hpp"

using namespace volcart::experimental::meshing;

ITK2Bullet::ITK2Bullet(
    ITKMesh::Pointer input, btSoftBodyWorldInfo& worldInfo, btSoftBody** output)
{

    for (auto it = input->GetPoints()->Begin(); it != input->GetPoints()->End();
         ++it) {
        // copy vertex info from itk mesh to btSoftBody
        auto oldPoint = it->Value();
        auto newPoint = new btVector3(
            static_cast<float>(oldPoint[0]), static_cast<float>(oldPoint[1]),
            static_cast<float>(oldPoint[2]));

        if (it == input->GetPoints()->Begin()) {
            *output = new btSoftBody(&worldInfo, 1, newPoint, nullptr);
        } else {
            (*output)->appendNode(*newPoint, 0);
        }
    }

    // convert the cells to faces
    uint64_t v0 = 0, v1 = 0, v2 = 0;

    for (auto cell = input->GetCells()->Begin();
         cell != input->GetCells()->End(); ++cell) {
        v0 = cell.Value()->GetPointIds()[0];
        v1 = cell.Value()->GetPointIds()[1];
        v2 = cell.Value()->GetPointIds()[2];

        (*output)->appendLink(v0, v1);
        (*output)->appendLink(v1, v2);
        (*output)->appendLink(v2, v0);

        (*output)->appendFace(v0, v1, v2);
    }
}

Bullet2ITK::Bullet2ITK(btSoftBody* input, ITKMesh::Pointer output)
{
    ITKCell::CellAutoPointer cellpointer;
    ITKPoint p;
    ITKPixel n;

    // iterate through points of bullet mesh (softBody)
    for (int i = 0; i < input->m_nodes.size(); ++i) {

        p[0] = input->m_nodes[i].m_x.x();
        p[1] = input->m_nodes[i].m_x.y();
        p[2] = input->m_nodes[i].m_x.z();

        n[0] = input->m_nodes[i].m_n.x();
        n[1] = input->m_nodes[i].m_n.y();
        n[2] = input->m_nodes[i].m_n.z();

        output->SetPoint(i, p);
        output->SetPointData(i, n);
    }
}
