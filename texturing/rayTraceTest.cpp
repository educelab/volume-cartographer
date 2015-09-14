// rayTraceTest.cpp
// Abigail Coleman Jul. 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "vc_defines.h"

int main(int argc, char* argv[]) {
    VC_MeshType::Pointer mesh = VC_MeshType::New();
    // Hardcode points to be used to make the cell/face
    VC_PointType p1, p2, p3;
    p1[0] = 50;
    p1[1] = 0;
    p1[2] = 20;
    mesh->SetPoint( 0, p1 );
    p2[0] = 50;
    p2[1] = 20;
    p2[2] = -20;
    mesh->SetPoint( 1, p2 );
    p3[0] = 50;
    p3[1] = -20;
    p3[2] = -20;
    mesh->SetPoint( 2, p3 );

    // Create cell/face
    VC_CellType::CellAutoPointer cellpointer;
    cellpointer.TakeOwnership( new VC_TriangleType );
    cellpointer->SetPointId( 0, 0 );
    cellpointer->SetPointId( 1, 1 );
    cellpointer->SetPointId( 2, 2 );
    mesh->SetCell( 0, cellpointer );

    VC_CellIterator cellIterator = mesh->GetCells()->Begin();
    VC_CellType* cell = cellIterator.Value();
    float origin[] = {0, 0, 0};
    float direction[] = {1, 0, 0}; // Along the x axis
    
    // Variables to store intersecting results into
    float result[3];
    float  t;
    float  intersection[3];

    // Check for intersection using itk cell method IntersectWithLine
    if( cell->IntersectWithLine(origin, direction, 0.00001, result, &t, intersection)){
        std::cout << "The cell intersected with at: " << intersection <<  std::endl;
    }
    else {
        std::cout << "No intersection, values returned: ";
        std::cout << intersection << ", " << intersection[1] << ", " << intersection[2] << " | " << t << std::endl;
    }
    return 0;
}
