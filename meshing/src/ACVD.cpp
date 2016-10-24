
/** @file ACVD.cpp */

// ACVD
// This is a refactor of the ACVD implementation found in the ACVD.cxx example
// of https://github.com/valette/ACVD
// This function is essentially a wrapper around that functionality
//
// This implements the iterative process discussed in:
//      Valette, Sébastien, and Jean‐Marc Chassery. "Approximated centroidal
//      voronoi diagrams for uniform polygonal mesh coarsening." Computer
//      Graphics Forum. Vol. 23. No. 3. Blackwell Publishing, Inc, 2004.
//
// It iteratively loops over the mesh until the approximated centroidal voronoi
// diagrams for the mesh are approximately
// equivalent in area. It then takes the point on the mesh that is nearest to
// the centroid of each diagram as a new point
// in the resampled output mesh.

#include "meshing/ACVD.h"

namespace volcart
{
namespace meshing
{
void ACVD(
    vtkPolyData* inputMesh,
    vtkPolyData* outputMesh,
    int numberOfSamples,
    float gradation,
    int consoleOutput,
    int subsamplingThreshold)
{

    // ACVD's vtkSurface class used for resampling
    vtkSurface* Mesh = vtkSurface::New();
    Mesh->CreateFromPolyData(inputMesh);
    Mesh->GetCellData()->Initialize();
    Mesh->GetPointData()->Initialize();

    ///// ACVD /////
    // initialize paramaters needed for ACVD resampling
    int QuadricsOptimizationLevel = 1;

    vtkIsotropicDiscreteRemeshing* Remesh =
        vtkIsotropicDiscreteRemeshing::New();

    Remesh->SetInput(Mesh);
    Remesh->SetFileLoadSaveOption(0);
    Remesh->SetNumberOfClusters(numberOfSamples);
    Remesh->SetConsoleOutput(consoleOutput);
    Remesh->SetSubsamplingThreshold(subsamplingThreshold);
    Remesh->GetMetric()->SetGradation(gradation);
    Remesh->Remesh();

    if (QuadricsOptimizationLevel != 0) {
        // Note : this is an adaptation of Siggraph 2000 Paper :
        // Out-of-core simplification of large polygonal models
        vtkIntArray* Clustering = Remesh->GetClustering();

        int Cluster, NumberOfMisclassedItems = 0;

        double** ClustersQuadrics = new double*[numberOfSamples];
        for (int i = 0; i < numberOfSamples; i++) {
            ClustersQuadrics[i] = new double[9];
            for (int j = 0; j < 9; j++) {
                ClustersQuadrics[i][j] = 0;
            }
        }
        vtkIdList* FList = vtkIdList::New();
        for (int i = 0; i < Remesh->GetNumberOfItems(); i++) {
            Cluster = Clustering->GetValue(i);
            if ((Cluster >= 0) && (Cluster < numberOfSamples)) {
                if (Remesh->GetClusteringType() == 0) {
                    vtkQuadricTools::AddTriangleQuadric(
                        ClustersQuadrics[Cluster], Remesh->GetInput(), i,
                        false);
                } else {
                    Remesh->GetInput()->GetVertexNeighbourFaces(i, FList);
                    for (int j = 0; j < FList->GetNumberOfIds(); j++)
                        vtkQuadricTools::AddTriangleQuadric(
                            ClustersQuadrics[Cluster], Remesh->GetInput(),
                            FList->GetId(j), false);
                }
            } else {
                NumberOfMisclassedItems++;
            }
        }
        FList->Delete();

        if (NumberOfMisclassedItems) {
            std::cout << NumberOfMisclassedItems
                      << " Items with wrong cluster association" << std::endl;
        }

        double P[3];
        for (int i = 0; i < numberOfSamples; i++) {
            Remesh->GetOutput()->GetPoint(i, P);
            vtkQuadricTools::ComputeRepresentativePoint(
                ClustersQuadrics[i], P, QuadricsOptimizationLevel);
            Remesh->GetOutput()->SetPointCoordinates(i, P);
            delete[] ClustersQuadrics[i];
        }
        delete[] ClustersQuadrics;

        Mesh->GetPoints()->Modified();

        // Disabled because all it does is show info about the remeshing. - SP
        // std::cout << "After Quadrics Post-processing : " << std::endl;
        // Remesh->GetOutput()->DisplayMeshProperties();
    }
    ///// End ACVD /////

    // Convert the vtkSurface back to vtkPolydata, regenerating normals as we go
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator =
        vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(Remesh->GetOutput());
    normalGenerator->SetOutput(outputMesh);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

}  // ACVD
}  // namespace meshing
}  // namespace volcart
