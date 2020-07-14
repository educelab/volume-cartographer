#include "vc/apps/render/RenderMeshing.hpp"

#include <boost/filesystem.hpp>
#include <vtkCleanPolyData.h>
#include <vtkSmoothPolyDataFilter.h>

#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/CalculateNormals.hpp"
#include "vc/meshing/ITK2VTK.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vcm = volcart::meshing;

extern po::variables_map parsed_;
extern vc::Volume::Pointer volume_;

// Square Micron to square millimeter conversion factor
static constexpr double UM_TO_MM = 0.000001;
// Min. number of points required to do flattening
static constexpr size_t CLEANER_MIN_REQ_POINTS = 100;
// Default resampling factor
static constexpr size_t DEFAULT_RESAMPLE_FACTOR = 50;

po::options_description GetMeshingOpts()
{
    // clang-format off
    po::options_description opts("Meshing Options");
    opts.add_options()
        ("scale-mesh", po::value<double>(), "Scale the mesh by a linear scale "
            "factor")
        ("enable-mesh-resampling", "Enable ACVD mesh resampling. Automatically "
            "enabled if the input is a Segmentation")
        ("mesh-resample-factor", po::value<double>()->default_value(DEFAULT_RESAMPLE_FACTOR),
         "Roughly, the number of vertices per square millimeter in the "
            "output mesh")
        ("mesh-resample-vcount", po::value<size_t>(), "The target number of vertices "
            "in the resampled mesh. If specified, overrides both the mesh-resample-factor "
            "and mesh-resample-keep-vcount options.")
        ("mesh-resample-keep-vcount", "If enabled, mesh resampling will "
            "attempt to maintain the number of vertices in the input mesh. "
            "Overrides the value set by --mesh-resample-factor.")
        ("mesh-resample-anisotropic", "Enable the anisotropic extension "
            "of the mesh resampler.")
        ("mesh-resample-gradation", po::value<double>()->default_value(0),
            "Set the resampling gradation constraint.")
        ("mesh-resample-quadrics-level", po::value<size_t>()->default_value(1),
            "Set the mesh resampling quadrics optimization level.")
        ("mesh-resample-smoothing", po::value<int>()->default_value(0),
            "Smoothing Options:\n"
         "  0 = Off\n"
         "  1 = Before mesh resampling\n"
         "  2 = After mesh resampling\n"
         "  3 = Both before and after mesh resampling")
        ("intermediate-mesh", po::value<std::string>(),"Output file path for the "
            "intermediate (i.e. scale + resampled) mesh. File is saved prior "
            "to flattening. Useful for testing meshing parameters.");
    // clang-format on

    return opts;
}

vc::ITKMesh::Pointer ResampleMesh(const vc::ITKMesh::Pointer& m)
{
    auto smooth =
        static_cast<SmoothOpt>(parsed_["mesh-resample-smoothing"].as<int>());

    ///// Resample the segmentation /////
    // Calculate sampling density
    size_t vertCount{0};
    auto voxelToMicron = volume_->voxelSize() * volume_->voxelSize();
    auto area = vc::meshmath::SurfaceArea(m) * voxelToMicron * UM_TO_MM;
    auto currentDensityFactor = m->GetNumberOfPoints() / area;
    double newDensityFactor;
    if (parsed_.count("mesh-resample-vcount")) {
        vertCount = parsed_["mesh-resample-vcount"].as<size_t>();
        newDensityFactor = vertCount / area;
    } else if (parsed_.count("mesh-resample-keep-vcount")) {
        newDensityFactor = currentDensityFactor;
        vertCount = static_cast<size_t>(m->GetNumberOfPoints());
    } else {
        newDensityFactor = parsed_["mesh-resample-factor"].as<double>();
        vertCount = static_cast<size_t>(newDensityFactor * area);
    }

    vertCount = (vertCount < CLEANER_MIN_REQ_POINTS) ? CLEANER_MIN_REQ_POINTS
                                                     : vertCount;
    // Copy the pointer to the input mesh
    auto workingMesh = m;

    // Pre-Smooth
    if (smooth == SmoothOpt::Both || smooth == SmoothOpt::Before) {
        // Convert to polydata
        auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
        vcm::ITK2VTK(workingMesh, vtkMesh);
        // Smoother
        std::cout << "Smoothing mesh..." << std::endl;
        auto vtkSmoother = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
        vtkSmoother->SetInputData(vtkMesh);
        vtkSmoother->Update();
        vtkMesh = vtkSmoother->GetOutput();
        // Convert back to ITK
        workingMesh = vc::ITKMesh::New();
        vcm::VTK2ITK(vtkMesh, workingMesh);
    }

    // Decimate using ACVD
    std::cout << "Resampling mesh ";
    std::cout << "(Density: " << currentDensityFactor;
    std::cout << " -> " << newDensityFactor << ")...";
    std::cout << std::endl;

    vcm::ACVD resampler;
    resampler.setInputMesh(workingMesh);
    resampler.setNumberOfClusters(vertCount);
    resampler.setGradation(parsed_["mesh-resample-gradation"].as<double>());
    resampler.setQuadricsOptimizationLevel(
        parsed_["mesh-resample-quadrics-level"].as<size_t>());
    if (parsed_.count("mesh-resample-anisotropic")) {
        resampler.setMode(vcm::ACVD::Mode::Anisotropic);
    }
    workingMesh = resampler.compute();

    // Post-Smooth
    if (smooth == SmoothOpt::Both || smooth == SmoothOpt::After) {
        // Convert to polydata
        auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
        vcm::ITK2VTK(workingMesh, vtkMesh);
        // Smoother
        std::cout << "Smoothing mesh..." << std::endl;
        auto vtkSmoother = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
        vtkSmoother->SetInputData(vtkMesh);
        vtkSmoother->Update();
        vtkMesh = vtkSmoother->GetOutput();
        // Convert back to ITK
        workingMesh = vc::ITKMesh::New();
        vcm::VTK2ITK(vtkMesh, workingMesh);
    }

    // Make sure the normals are up-to-date if we've smoothed
    if (smooth == SmoothOpt::Both || smooth == SmoothOpt::Before ||
        smooth == SmoothOpt::After) {
        vcm::CalculateNormals normals;
        normals.setMesh(workingMesh);
        workingMesh = normals.compute();
    }

    // Save the intermediate mesh
    if (parsed_.count("intermediate-mesh")) {
        std::cout << "Writing intermediate mesh..." << std::endl;
        fs::path meshPath = parsed_["intermediate-mesh"].as<std::string>();
        if (vc::io::FileExtensionFilter(meshPath, {"ply"})) {
            vc::io::PLYWriter writer;
            writer.setMesh(workingMesh);
            writer.setPath(meshPath);
            writer.write();
        } else if (vc::io::FileExtensionFilter(meshPath, {"obj"})) {
            vc::io::OBJWriter writer;
            writer.setMesh(workingMesh);
            writer.setPath(meshPath);
            writer.write();
        }
    }

    return workingMesh;
}
