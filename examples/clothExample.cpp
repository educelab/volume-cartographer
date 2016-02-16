//
// Created by Abigail Coleman 2/15/16
//

#include "volumepkg.h"
#include "cloth.h"

std::vector< btSoftBody::Node* > volcart::meshing::cloth::_pinnedPoints;
std::vector< volcart::meshing::cloth::NodeTarget > volcart::meshing::cloth::_targetPoints;

int main(int argc, char* argv[]) {
    if ( argc < 3 ) {
        std::cout << "Usage: vc_clothExample volpkg seg-id iterations" << std::endl;
        return EXIT_FAILURE;
    }

    VolumePkg vpkg = VolumePkg( argv[ 1 ] );
    std::string segID = argv[ 2 ];
    if (segID == "") {
    printf("ERROR: Incorrect/missing segmentation ID!\n");
    exit(EXIT_FAILURE);
    }
    if ( vpkg.getVersion() < 2.0) {
        printf("ERROR: Volume package is version %f but this program requires a version >= 2.0.\n", vpkg.getVersion() );
        exit(EXIT_FAILURE);
    }
    vpkg.setActiveSegmentation(segID);
    std::string meshName = vpkg.getMeshPath();

    int64_t NUM_OF_ITERATIONS = atoi( argv[ 3 ] );

    // declare pointer to new Mesh object
    VC_MeshType::Pointer  mesh = VC_MeshType::New();

    int meshWidth = -1;
    int meshHeight = -1;

    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( "decim.ply" );

    vtkSmoothPolyDataFilter* smoother = vtkSmoothPolyDataFilter::New();
    smoother->SetInputConnection( reader->GetOutputPort() );
    smoother->SetNumberOfIterations(3);
    smoother->SetRelaxationFactor(0.3);
    smoother->Update();

    VC_MeshType::Pointer decimated = VC_MeshType::New();
    volcart::meshing::vtk2itk(smoother->GetOutput(), decimated);
    std::cerr << "points: " << decimated->GetNumberOfPoints() << " || cells: " << decimated->GetNumberOfCells() << std::endl;

    // try to convert the ply to an ITK mesh
    if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)) {
        exit( -1 );
    };

    volcart::meshing::cloth Cloth = volcart::meshing::cloth(mesh, decimated, meshWidth, meshHeight, NUM_OF_ITERATIONS);

    volcart::UVMap uvMap = Cloth._returnUVMap();

    VC_MeshType::Pointer result = VC_MeshType::New();
    result = Cloth._returnMesh();

    return EXIT_SUCCESS;
}