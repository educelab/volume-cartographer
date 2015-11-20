//
// Created by Abigail Coleman 10/21/15
//

#include "itk2bullet.h"

namespace volcart {
  namespace meshing {

    itk2bullet::itk2bullet( VC_MeshType::Pointer input, btScalar vertices[], int faces[][3] ) {

    	int i = 0;
    	// copy point vertices from itk mesh to array of btScalar type
    	VC_PointsInMeshIterator point = input->GetPoints()->Begin();
    	VC_PointsInMeshIterator end = input->GetPoints()->End();
    	while (point != end) {
                
        VC_MeshType::PointType p = point.Value();

        // push vertices of point into array
        vertices[i] = ( btScalar(p[0]) );
        vertices[i+1] = ( btScalar(p[1]) );
        vertices[i+2] = ( btScalar(p[2]) );

        i += 3;
        ++point;
      }

      int j = 0;
    	// Iterate over all of the cells to copy the indices of each vertice to multidimensional int array
    	for ( VC_CellIterator cell = input->GetCells()->Begin(); cell != input->GetCells()->End(); ++cell ) {
        // Link the pointer to our current cell
        VC_CellType * c = cell.Value();
      	
      	int k = 0;
        // Iterate over the vertices of the current cell
        for ( VC_PointsInCellIterator cellPoint = c->PointIdsBegin(); cellPoint != c->PointIdsEnd(); ++cellPoint ) {

        	faces[j][k] = *cellPoint; // this gives the indice of the point, not the points coordinates

        	++k;
        }

        ++j;
      }

    };

    bullet2itk::bullet2itk( VC_MeshType::Pointer output, btSoftBody* softBody ) {

    	VC_CellType::CellAutoPointer cellpointer;
    	VC_PointType p;
    	VC_PixelType n;
    	int NUM_OF_CELLS = softBody->m_faces.size();

    	// iterate through faces of bullet mesh (softBody)
  		for(int i = 0; i < NUM_OF_CELLS; ++i) {

  			cellpointer.TakeOwnership( new VC_TriangleType );

    		for(int j = 0; j < 3; ++j) {

      		p[0] = softBody->m_faces[i].m_n[j]->m_x.x();
      		p[1] = softBody->m_faces[i].m_n[j]->m_x.y();
					p[2] = softBody->m_faces[i].m_n[j]->m_x.z();

					n[0] = softBody->m_faces[i].m_normal.x();
      		n[1] = softBody->m_faces[i].m_normal.y();
					n[2] = softBody->m_faces[i].m_normal.z();
					
					output->SetPoint( (i * 3 + j), p );
					output->SetPointData( (i * 3 + j), n );

					cellpointer->SetPointId( j, (i * 3 + j));
    		}

    		output->SetCell( i, cellpointer );
  		}
    };

  } // namespace meshing
} //  namespace volcart