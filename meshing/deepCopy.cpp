//
// Created by Seth Parker on 12/21/15.
//

#include "deepCopy.h"

namespace volcart {
    namespace meshing {
        deepCopy::deepCopy(VC_MeshType::Pointer input, VC_MeshType::Pointer output) {

            // Copy the points and their normals
            VC_PointType p;
            VC_PixelType n;
            for ( VC_PointsInMeshIterator pt = input->GetPoints()->Begin(); pt != input->GetPoints()->End(); ++pt ) {
                p = pt->Value();
                input->GetPointData(pt.Index(), &n);

                output->SetPoint( pt.Index(), p );
                output->SetPointData( pt.Index(), n );
            }

            // Copy the faces
            VC_CellType::CellAutoPointer c;
            for ( VC_CellIterator cell = input->GetCells()->Begin(); cell != input->GetCells()->End(); ++cell ) {

                c.TakeOwnership( new VC_TriangleType );
                for ( int p_id = 0; p_id < cell.Value()->GetNumberOfPoints(); ++p_id )
                    c->SetPointId(p_id, cell.Value()->GetPointIdsContainer()[p_id] );

                output->SetCell( cell->Index(), c );
            }
        } // deepCopy
    } // meshing
} // volcart