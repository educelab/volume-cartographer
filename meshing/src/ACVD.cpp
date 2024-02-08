#include "vc/meshing/ACVD.hpp"

#include <array>

#include <vtkAnisotropicDiscreteRemeshing.h>
#include <vtkCleanPolyData.h>
#include <vtkIsotropicDiscreteRemeshing.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>

#include "vc/core/util/Logging.hpp"
#include "vc/meshing/ITK2VTK.hpp"

using namespace volcart;
using namespace volcart::meshing;

void ACVD::setInputMesh(ITKMesh::Pointer input)
{
    inputMesh_ = std::move(input);
}

void ACVD::setMode(Mode m) { mode_ = m; }

void ACVD::setNumberOfClusters(std::size_t n) { clusters_ = n; }

void ACVD::setGradation(double g) { gradation_ = g; }

void ACVD::setSubsampleThreshold(std::size_t t) { subsampleThreshold_ = t; }

void ACVD::setQuadricsOptimizationLevel(std::size_t l)
{
    quadricsOptLevel_ = l;
}

auto ACVD::getOutputMesh() const -> ITKMesh::Pointer { return outputMesh_; }

auto ACVD::compute() -> ITKMesh::Pointer
{
    Logger()->info(
        "ACVD: Input: {} verts, {} faces", inputMesh_->GetNumberOfPoints(),
        inputMesh_->GetNumberOfCells());
    switch (mode_) {
        case Mode::Isotropic:
            compute_isotropic_();
            break;
        case Mode::Anisotropic:
            compute_anisotropic_();
            break;
    }
    Logger()->info(
        "ACVD: Output: {} verts, {} faces", outputMesh_->GetNumberOfPoints(),
        outputMesh_->GetNumberOfCells());
    return outputMesh_;
}

void ACVD::compute_isotropic_()
{
    // Convert to polydata
    vtkNew<vtkPolyData> vtkMesh;
    ITK2VTK(inputMesh_, vtkMesh);

    // ACVD's vtkSurface class used for resampling
    vtkNew<vtkSurface> mesh;
    mesh->CreateFromPolyData(vtkMesh);
    mesh->GetCellData()->Initialize();
    mesh->GetPointData()->Initialize();

    // Validate clusters parameter
    auto clusters = clusters_;
    if (clusters_ == 0) {
        clusters = inputMesh_->GetNumberOfPoints();
    }

    // Run ACVD
    Logger()->info("ACVD: Performing isotropic mesh resampling...");
    vtkNew<vtkIsotropicDiscreteRemeshing> remesh;
    remesh->SetInput(mesh);
    remesh->SetNumberOfClusters(static_cast<int>(clusters));
    remesh->SetSubsamplingThreshold(static_cast<int>(subsampleThreshold_));
    remesh->GetMetric()->SetGradation(gradation_);
    remesh->Remesh();

    // Note : this is an adaptation of Siggraph 2000 Paper :
    // Out-of-core simplification of large polygonal models
    // Use quadrics error to minimize distance between input and resampled
    if (quadricsOptLevel_ != 0) {
        Logger()->info("ACVD: Computing quadrics optimization...");
        vtkIntArray* clustering = remesh->GetClustering();

        int cluster = 0;
        int numMisclassedItems = 0;

        std::vector<std::array<double, 9>> clusterQuadrics(
            clusters, std::array<double, 9>{});
        auto* fList = vtkIdList::New();

        for (int i = 0; i < remesh->GetNumberOfItems(); i++) {
            cluster = clustering->GetValue(i);
            if (cluster >= 0 && cluster < clusters) {
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
            Logger()->warn(
                "ACVD: Items with wrong cluster association: {}",
                numMisclassedItems);
        }

        std::array<double, 3> pt{};
        for (int idx = 0; idx < clusters; idx++) {
            remesh->GetOutput()->GetPoint(idx, pt.data());
            vtkQuadricTools::ComputeRepresentativePoint(
                clusterQuadrics[idx].data(), pt.data(),
                static_cast<int>(quadricsOptLevel_));
            remesh->GetOutput()->SetPointCoordinates(idx, pt.data());
        }

        mesh->GetPoints()->Modified();
    }

    // Convert the vtkSurface back to vtkPolydata, regenerating normals as we go
    vtkNew<vtkPolyData> vtkOut;
    auto normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(remesh->GetOutput());
    normalGenerator->SetOutput(vtkOut);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

    // Remove duplicate vertices
    vtkNew<vtkCleanPolyData> cleaner;
    cleaner->SetInputData(vtkOut);
    cleaner->Update();

    outputMesh_ = ITKMesh::New();
    VTK2ITK(cleaner->GetOutput(), outputMesh_);
}

void ACVD::compute_anisotropic_()
{
    // Convert to polydata
    vtkNew<vtkPolyData> vtkMesh;
    ITK2VTK(inputMesh_, vtkMesh);

    vtkNew<vtkSurface> mesh;
    mesh->CreateFromPolyData(vtkMesh);
    mesh->GetCellData()->Initialize();
    mesh->GetPointData()->Initialize();

    // Validate clusters parameter
    auto clusters = clusters_;
    if (clusters_ == 0) {
        clusters = inputMesh_->GetNumberOfPoints();
    }

    // Run remeshing
    Logger()->info("ACVD: Performing anisotropic mesh resampling...");
    vtkNew<vtkAnisotropicDiscreteRemeshing> remesh;
    remesh->SetInput(mesh);
    remesh->SetNumberOfClusters(static_cast<int>(clusters));
    remesh->SetSubsamplingThreshold(static_cast<int>(subsampleThreshold_));
    remesh->GetMetric()->SetGradation(gradation_);
    remesh->Remesh();

    // Convert the vtkSurface back to vtkPolydata, regenerating normals as we go
    vtkNew<vtkPolyData> vtkOut;
    auto normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(remesh->GetOutput());
    normalGenerator->SetOutput(vtkOut);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

    // Remove duplicate vertices
    vtkNew<vtkCleanPolyData> cleaner;
    cleaner->SetInputData(vtkOut);
    cleaner->Update();

    outputMesh_ = ITKMesh::New();
    VTK2ITK(cleaner->GetOutput(), outputMesh_);
}

auto ACVD::mode() const -> ACVD::Mode { return mode_; }

auto ACVD::numberOfClusters() const -> std::size_t { return clusters_; }

auto ACVD::gradation() const -> double { return gradation_; }

auto ACVD::subsampleThreshold() const -> std::size_t
{
    return subsampleThreshold_;
}

auto ACVD::quadricsOptimizationLevel() const -> std::size_t
{
    return quadricsOptLevel_;
}
