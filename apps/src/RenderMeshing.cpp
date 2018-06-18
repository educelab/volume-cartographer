#include "apps/RenderMeshing.hpp"

#include <vtkCleanPolyData.h>
#include <vtkSmoothPolyDataFilter.h>

#include "vc/core/types/Volume.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/CalculateNormals.hpp"
#include "vc/meshing/ITK2VTK.hpp"

namespace po = boost::program_options;
namespace vc = volcart;
namespace vcm = volcart::meshing;

extern po::variables_map parsed_;
extern vc::Volume::Pointer volume_;

// Square Micron to square millimeter conversion factor
static constexpr double UM_TO_MM = 0.001 * 0.001;
// Min. number of points required to do flattening
static constexpr int CLEANER_MIN_REQ_POINTS = 100;

po::options_description GetMeshingOpts()
{
    // clang-format off
    po::options_description opts("Meshing Options");
    opts.add_options()
        ("scale-mesh", po::value<double>(), "Scale the mesh by a linear scale "
            "factor")
        ("enable-mesh-resampling", "Enable ACVD mesh resampling. Automatically "
            "enabled if the input is a Segmentation")
        ("mesh-resample-factor", po::value<double>()->default_value(50),
         "Roughly, the number of vertices per square millimeter in the "
            "output mesh")
        ("mesh-resample-keep-vcount", "If enabled, mesh resampling will "
            "attempt to maintain the number of vertices in the input mesh. "
            "Overrides the value set by --mesh-resample-factor.")
        ("mesh-resample-smoothing", po::value<int>()->default_value(0),
            "Smoothing Options:\n"
         "  0 = Off\n"
         "  1 = Before mesh resampling\n"
         "  2 = After mesh resampling\n"
         "  3 = Both before and after mesh resampling");
    // clang-format on

    return opts;
}

vc::ITKMesh::Pointer ResampleMesh(const vc::ITKMesh::Pointer& m)
{
    auto smooth =
        static_cast<SmoothOpt>(parsed_["mesh-resample-smoothing"].as<int>());

    ///// Resample the segmentation /////
    // Calculate sampling density
    int vertCount{0};
    auto voxelToMicron = std::pow(volume_->voxelSize(), 2);
    auto area = vc::meshmath::SurfaceArea(m) * voxelToMicron * UM_TO_MM;
    auto currentDensityFactor = m->GetNumberOfPoints() / area;
    double newDensityFactor;
    if (parsed_.count("mesh-resample-keep-vcount")) {
        newDensityFactor = currentDensityFactor;
        vertCount = static_cast<int>(m->GetNumberOfPoints());
    } else {
        newDensityFactor = parsed_["mesh-resample-factor"].as<double>();
        vertCount = static_cast<int>(newDensityFactor * area);
    }

    vertCount = (vertCount < CLEANER_MIN_REQ_POINTS) ? CLEANER_MIN_REQ_POINTS
                                                     : vertCount;
    // Convert to polydata
    auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
    vcm::ITK2VTK(m, vtkMesh);

    // Pre-Smooth
    if (smooth == SmoothOpt::Both || smooth == SmoothOpt::Before) {
        std::cout << "Smoothing mesh..." << std::endl;
        auto vtkSmoother = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
        vtkSmoother->SetInputData(vtkMesh);
        vtkSmoother->Update();
        vtkMesh = vtkSmoother->GetOutput();
    }

    // Decimate using ACVD
    std::cout << "Resampling mesh (Density: " << currentDensityFactor << " -> "
              << newDensityFactor << ")..." << std::endl;
    auto acvdMesh = vtkSmartPointer<vtkPolyData>::New();
    vcm::ACVD(vtkMesh, acvdMesh, vertCount);

    // Merge Duplicates
    // Note: This merging has to be the last in the process chain for some
    // really weird reason. - SP
    auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
    cleaner->SetInputData(acvdMesh);
    cleaner->Update();
    vtkMesh = cleaner->GetOutput();

    // Post-Smooth
    if (smooth == SmoothOpt::Both || smooth == SmoothOpt::After) {
        std::cout << "Smoothing mesh..." << std::endl;
        auto vtkSmoother = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
        vtkSmoother->SetInputData(vtkMesh);
        vtkSmoother->Update();
        vtkMesh = vtkSmoother->GetOutput();
    }

    auto itkACVD = vc::ITKMesh::New();
    vcm::VTK2ITK(vtkMesh, itkACVD);

    // Make sure the normals are up-to-date if we've smoothed
    if (smooth == SmoothOpt::Both || smooth == SmoothOpt::Before ||
        smooth == SmoothOpt::After) {
        vcm::CalculateNormals normals(itkACVD);
        itkACVD = normals.compute();
    }

    return itkACVD;
}
