/** @file ACVD.cpp */

// ACVD
// This is a refactor of the ACVD implementation found in the ACVD.cxx example
// of https://github.com/valette/ACVD
// This function is essentially a wrapper around that functionality
//
// This implements the iterative process discussed in:
//      Valette, Sebastien, and Jean-Marc Chassery. "Approximated centroidal
//      voronoi diagrams for uniform polygonal mesh coarsening." Computer
//      Graphics Forum. Vol. 23. No. 3. Blackwell Publishing, Inc, 2004.
//
// It iteratively loops over the mesh until the approximated centroidal voronoi
// diagrams for the mesh are approximately
// equivalent in area. It then takes the point on the mesh that is nearest to
// the centroid of each diagram as a new point
// in the resampled output mesh.

#include <vtkPolyDataNormals.h>
#include "vtkIsotropicDiscreteRemeshing.h"

#include "meshing/ACVD.h"
#include "meshing/ITK2VTK.h"

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
    auto mesh = vtkSurface::New();
    mesh->CreateFromPolyData(inputMesh);
    mesh->GetCellData()->Initialize();
    mesh->GetPointData()->Initialize();

    ///// ACVD /////
    // initialize paramaters needed for ACVD resampling
    int quadricsOptimizationLevel = 1;

    auto remesh = vtkIsotropicDiscreteRemeshing::New();

    remesh->SetInput(mesh);
    remesh->SetFileLoadSaveOption(0);
    remesh->SetNumberOfClusters(numberOfSamples);
    remesh->SetConsoleOutput(consoleOutput);
    remesh->SetSubsamplingThreshold(subsamplingThreshold);
    remesh->GetMetric()->SetGradation(gradation);
    remesh->Remesh();

    if (quadricsOptimizationLevel != 0) {
        // Note : this is an adaptation of Siggraph 2000 Paper :
        // Out-of-core simplification of large polygonal models
        auto clustering = remesh->GetClustering();

        int cluster, numMisclassedItems = 0;

        std::vector<std::array<double, 9>> clusterQuadrics(
            numberOfSamples, std::array<double, 9>{});
        auto fList = vtkIdList::New();

        for (int i = 0; i < remesh->GetNumberOfItems(); i++) {
            cluster = clustering->GetValue(i);
            if (cluster >= 0 && cluster < numberOfSamples) {
                if (remesh->GetClusteringType() == 0) {
                    vtkQuadricTools::AddTriangleQuadric(
                        clusterQuadrics[cluster].data(), remesh->GetInput(), i,
                        false);
                } else {
                    remesh->GetInput()->GetVertexNeighbourFaces(i, fList);
                    for (int j = 0; j < fList->GetNumberOfIds(); j++) {
                        vtkQuadricTools::AddTriangleQuadric(
                            clusterQuadrics[cluster].data(), remesh->GetInput(),
                            fList->GetId(j), false);
                    }
                }
            } else {
                numMisclassedItems++;
            }
        }
        fList->Delete();

        if (numMisclassedItems != 0) {
            std::cout << numMisclassedItems
                      << " Items with wrong cluster association" << std::endl;
        }

        std::array<double, 3> p{};
        for (int i = 0; i < numberOfSamples; i++) {
            remesh->GetOutput()->GetPoint(i, p.data());
            vtkQuadricTools::ComputeRepresentativePoint(
                clusterQuadrics[i].data(), p.data(), quadricsOptimizationLevel);
            remesh->GetOutput()->SetPointCoordinates(i, p.data());
        }

        mesh->GetPoints()->Modified();

        // Disabled because all it does is show info about the remeshing. - SP
        // std::cout << "After Quadrics Post-processing : " << std::endl;
        // Remesh->GetOutput()->DisplayMeshProperties();
    }
    ///// End ACVD /////

    // Convert the vtkSurface back to vtkPolydata, regenerating normals as we go
    auto normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(remesh->GetOutput());
    normalGenerator->SetOutput(outputMesh);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();
}
}
}
