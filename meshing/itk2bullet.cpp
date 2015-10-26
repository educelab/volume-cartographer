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

     //  std::cout << "VERTICES CHECK" << std::endl;

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

  } // namespace meshing
} //  namespace volcart