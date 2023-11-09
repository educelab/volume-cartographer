#include "vc/graph/meshing.hpp"

#include <nlohmann/json.hpp>

#include "vc/core/io/MeshIO.hpp"
#include "vc/core/util/Json.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/meshing/ScaleMesh.hpp"

using namespace volcart;
using namespace volcart::meshing;
namespace fs = volcart::filesystem;

// Enum conversions
namespace volcart::meshing
{
// clang-format off
using ResampleMode = ResampleMeshNode::Mode;
NLOHMANN_JSON_SERIALIZE_ENUM(ResampleMode, {
    {ResampleMode::Isotropic, "isotropic"},
    {ResampleMode::Anisotropic, "anisotropic"}
})

using ReferenceMode = OrientNormalsNode::ReferenceMode;
NLOHMANN_JSON_SERIALIZE_ENUM(ReferenceMode , {
    {ReferenceMode::Centroid, "centroid"},
    {ReferenceMode::Manual, "manual"}
})
// clang-format on
}  // namespace volcart::meshing

MeshingNode::MeshingNode()
    : Node{true}, points{&mesher_, &Mesher::setPointSet}, mesh{&mesh_}
{
    registerInputPort("points", points);
    registerOutputPort("mesh", mesh);
    compute = [&]() {
        Logger()->debug("[graph.meshing] meshing pointset");
        mesh_ = mesher_.compute();
    };
}

auto MeshingNode::serialize_(bool useCache, const fs::path& cacheDir)
    -> smgl::Metadata
{
    smgl::Metadata meta;
    if (useCache and mesh_) {
        WriteMesh(cacheDir / "mesh.obj", mesh_);
        meta["mesh"] = "mesh.obj";
    }
    return meta;
}

void MeshingNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    if (meta.contains("mesh")) {
        auto meshFile = meta["mesh"].get<std::string>();
        mesh_ = ReadMesh(cacheDir / meshFile).mesh;
    }
}

ScaleMeshNode::ScaleMeshNode()
    : Node{true}, input{&input_}, scaleFactor{&scaleFactor_}, output{&output_}
{
    registerInputPort("input", input);
    registerInputPort("scaleFactor", scaleFactor);
    registerOutputPort("output", output);

    compute = [&]() {
        if (input_) {
            Logger()->debug(
                "[graph.meshing] scaling mesh {.3f}x", scaleFactor_);
            output_ = ScaleMesh(input_, scaleFactor_);
        }
    };
}

auto ScaleMeshNode::serialize_(bool useCache, const fs::path& cacheDir)
    -> smgl::Metadata
{
    smgl::Metadata meta{{"scaleFactor", scaleFactor_}};
    if (useCache and output_) {
        WriteMesh(cacheDir / "scaled.obj", output_);
        meta["output"] = "scaled.obj";
    }
    return meta;
}

void ScaleMeshNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    scaleFactor_ = meta["scaleFactor"].get<double>();
    if (meta.contains("output")) {
        auto meshFile = meta["output"].get<std::string>();
        output_ = ReadMesh(cacheDir / meshFile).mesh;
    }
}

CalculateNumVertsNode::CalculateNumVertsNode()
    : mesh{&mesh_}
    , voxelSize{&voxelSize_}
    , density{&density_}
    , numVerts{&numVerts_}
{
    registerInputPort("mesh", mesh);
    registerInputPort("voxelSize", voxelSize);
    registerInputPort("density", density);
    registerOutputPort("numVerts", numVerts);

    compute = [&]() {
        Logger()->debug("[graph.meshing] calculating number of vertices");
        using meshmath::SurfaceArea;
        static constexpr double UM_TO_MM{0.000001};
        static constexpr std::size_t MIN_NUM{100};
        auto sqVoxel = voxelSize_ * voxelSize_;
        auto a = SurfaceArea(mesh_) * sqVoxel * UM_TO_MM;
        numVerts_ = std::max(static_cast<size_t>(density_ * a), MIN_NUM);
    };
}

smgl::Metadata CalculateNumVertsNode::serialize_(
    bool /*useCache*/, const fs::path& /*cacheDir*/)
{
    return {
        {"density", density_},
        {"numVerts", numVerts_},
        {"voxelSize", voxelSize_}};
}

void CalculateNumVertsNode::deserialize_(
    const smgl::Metadata& meta, const fs::path&)
{
    density_ = meta["density"].get<double>();
    voxelSize_ = meta["voxelSize"].get<double>();
    numVerts_ = meta["numVerts"].get<size_t>();
}

LaplacianSmoothMeshNode::LaplacianSmoothMeshNode()
    : Node{true}, input{&smoother_, &Smoother::setInputMesh}, output{&mesh_}
{
    registerInputPort("input", input);
    registerOutputPort("output", output);
    compute = [&]() {
        Logger()->debug("[graph.meshing] smoothing mesh");
        mesh_ = smoother_.compute();
    };
}

auto LaplacianSmoothMeshNode::serialize_(
    bool useCache, const fs::path& cacheDir) -> smgl::Metadata
{
    smgl::Metadata meta{
        {"iterations", smoother_.iterations()},
        {"relaxationFactor", smoother_.relaxationFactor()},
        {"featureEdgeSmoothing", smoother_.featureEdgeSmoothing()},
        {"featureAngle", smoother_.featureAngle()},
        {"edgeAngle", smoother_.edgeAngle()},
        {"boundarySmoothing", smoother_.boundarySmoothing()}};
    if (useCache and mesh_) {
        WriteMesh(cacheDir / "smoothed.obj", mesh_);
        meta["mesh"] = "smoothed.obj";
    }
    return meta;
}

void LaplacianSmoothMeshNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    smoother_.setIterations(meta["iterations"].get<std::size_t>());
    smoother_.setRelaxationFactor(meta["relaxationFactor"].get<double>());
    smoother_.setFeatureEdgeSmoothing(meta["featureEdgeSmoothing"].get<bool>());
    smoother_.setFeatureAngle(meta["featureAngle"].get<double>());
    smoother_.setEdgeAngle(meta["edgeAngle"].get<double>());
    smoother_.setBoundarySmoothing(meta["boundarySmoothing"].get<bool>());

    if (meta.contains("mesh")) {
        auto meshFile = meta["mesh"].get<std::string>();
        mesh_ = ReadMesh(cacheDir / meshFile).mesh;
    }
}

ResampleMeshNode::ResampleMeshNode()
    : Node{true}
    , input{&acvd_, &ACVD::setInputMesh}
    , mode{&acvd_, &ACVD::setMode}
    , numVertices{&acvd_, &ACVD::setNumberOfClusters}
    , gradation{&acvd_, &ACVD::setGradation}
    , subsampleThreshold{&acvd_, &ACVD::setSubsampleThreshold}
    , quadricsOptimizationLevel{&acvd_, &ACVD::setQuadricsOptimizationLevel}
    , output{&mesh_}
{
    registerInputPort("input", input);
    registerInputPort("mode", mode);
    registerInputPort("numVertices", numVertices);
    registerInputPort("gradation", gradation);
    registerInputPort("subsampleThreshold", subsampleThreshold);
    registerInputPort("quadricsOptimizationLevel", quadricsOptimizationLevel);
    registerOutputPort("output", output);
    compute = [&]() {
        Logger()->debug(
            "[graph.meshing] resampling mesh to {} vertices",
            acvd_.numberOfClusters());
        mesh_ = acvd_.compute();
    };
}

auto ResampleMeshNode::serialize_(bool useCache, const fs::path& cacheDir)
    -> smgl::Metadata
{
    smgl::Metadata meta{
        {"mode", acvd_.mode()},
        {"numVertices", acvd_.numberOfClusters()},
        {"gradation", acvd_.gradation()},
        {"subsampleThreshold", acvd_.subsampleThreshold()},
        {"quadricsOptimizationLevel", acvd_.quadricsOptimizationLevel()}};
    if (useCache and mesh_) {
        WriteMesh(cacheDir / "resampled.obj", mesh_);
        meta["mesh"] = "resampled.obj";
    }
    return meta;
}

void ResampleMeshNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    acvd_.setMode(meta["mode"].get<Mode>());
    acvd_.setNumberOfClusters(meta["numVertices"].get<std::size_t>());
    acvd_.setGradation(meta["gradation"].get<double>());
    acvd_.setSubsampleThreshold(meta["subsampleThreshold"].get<std::size_t>());
    acvd_.setQuadricsOptimizationLevel(
        meta["quadricsOptimizationLevel"].get<std::size_t>());

    if (meta.contains("mesh")) {
        auto meshFile = meta["mesh"].get<std::string>();
        mesh_ = ReadMesh(cacheDir / meshFile).mesh;
    }
}

UVMapToMeshNode::UVMapToMeshNode()
    : Node{true}
    , inputMesh{&mesher_, &UVMapToITKMesh::setMesh}
    , uvMap{&mesher_, &UVMapToITKMesh::setUVMap}
    , scaleToUVDimensions{&scaleDims_}
    , outputMesh{&output_}
{
    registerInputPort("inputMesh", inputMesh);
    registerInputPort("uvMap", uvMap);
    registerInputPort("scaleToUVDimensions", scaleToUVDimensions);
    registerOutputPort("outputMesh", outputMesh);

    compute = [&]() {
        Logger()->debug("[graph.meshing] converting UV map to mesh");
        mesher_.setScaleToUVDimensions(scaleDims_);
        output_ = mesher_.compute();
    };
}

auto UVMapToMeshNode::serialize_(bool useCache, const fs::path& cacheDir)
    -> smgl::Metadata
{
    smgl::Metadata meta{{"scaleToUVDims", scaleDims_}};
    if (useCache and output_) {
        WriteMesh(cacheDir / "uvMesh.obj", output_);
        meta["mesh"] = "uvMesh.obj";
    }
    return meta;
}

void UVMapToMeshNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    scaleDims_ = meta["scaleToUVDims"].get<bool>();
    if (meta.contains("mesh")) {
        auto file = meta["mesh"].get<std::string>();
        output_ = ReadMesh(cacheDir / file).mesh;
    }
}

OrientNormalsNode::OrientNormalsNode()
    : Node{true}
    , input{&orientNormals_, &OrientNormals::setMesh}
    , referenceMode{&orientNormals_, &OrientNormals::setReferenceMode}
    , referencePoint{&orientNormals_, &OrientNormals::setReferencePoint}
    , output{&output_}
{
    registerInputPort("input", input);
    registerInputPort("referenceMode", referenceMode);
    registerInputPort("referencePoint", referencePoint);
    registerOutputPort("output", output);
    compute = [&]() {
        Logger()->debug("[graph.meshing] orienting vertex normals");
        output_ = orientNormals_.compute();
    };
}

auto OrientNormalsNode::serialize_(bool useCache, const fs::path& cacheDir)
    -> smgl::Metadata
{
    smgl::Metadata meta{{"referenceMode", orientNormals_.referenceMode()}};
    if (orientNormals_.referenceMode() == ReferenceMode::Manual) {
        meta["referencePoint"] = orientNormals_.referencePoint();
    }
    if (useCache and output_) {
        WriteMesh(cacheDir / "orient_normals.obj", output_);
        meta["mesh"] = "orient_normals.obj";
    }
    return meta;
}

void OrientNormalsNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    auto refMode = meta["referenceMode"].get<ReferenceMode>();
    orientNormals_.setReferenceMode(refMode);
    if (refMode == ReferenceMode::Manual) {
        orientNormals_.setReferencePoint(
            meta["referencePoint"].get<cv::Vec3d>());
    }

    if (meta.contains("mesh")) {
        auto meshFile = meta["mesh"].get<std::string>();
        output_ = ReadMesh(cacheDir / meshFile).mesh;
    }
}