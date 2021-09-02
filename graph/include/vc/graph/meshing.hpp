#pragma once

/** @file */

#include <smgl/Node.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/LaplacianSmooth.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"
#include "vc/meshing/UVMapToITKMesh.hpp"

namespace volcart
{

/**
 * @copybrief meshing::OrderedPointSetMesher
 *
 * @see meshing::OrderedPointSetMesher
 * @ingroup Graph
 */
class MeshingNode : public smgl::Node
{
private:
    /** Meshing class type */
    using Mesher = meshing::OrderedPointSetMesher;
    /** Meshing */
    Mesher mesher_;
    /** Input mesh */
    ITKMesh::Pointer mesh_;

public:
    /** @brief Input point set */
    smgl::InputPort<Mesher::PointSet> points;
    /** @brief Output mesh */
    smgl::OutputPort<ITKMesh::Pointer> mesh;

    /** Constructor */
    MeshingNode();

private:
    /** Smeagol custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** Smeagol custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @copydoc meshing::ScaleMesh(const ITKMesh::Pointer&, double)
 *
 * @see meshing::ScaleMesh(const ITKMesh::Pointer&, double)
 * @ingroup Graph
 */
class ScaleMeshNode : public smgl::Node
{
private:
    /** Input mesh */
    ITKMesh::Pointer input_;
    /** Linear scale factor */
    double scaleFactor_{1};
    /** Output mesh */
    ITKMesh::Pointer output_;

public:
    /** @brief Input mesh */
    smgl::InputPort<ITKMesh::Pointer> input;
    /** @brief Linear scale factor */
    smgl::InputPort<double> scaleFactor;
    /** @brief Scaled output mesh */
    smgl::OutputPort<ITKMesh::Pointer> output;

    /** Constructor */
    ScaleMeshNode();

private:
    /** Smeagol custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** Smeagol custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @brief Calculate the number of vertices required to produce a particular
 * resampling density factor
 *
 * Density factor is defined as the average number of vertices per square mm of
 * mesh surface area. This class calculates the number of vertices needed to
 * produce a given density factor for a particular mesh. The number of vertices
 * produced bu this class are meant to be passed to
 * ResampleMeshNode::numVertices.
 *
 * @ingroup Graph
 */
class CalculateNumVertsNode : public smgl::Node
{
private:
    /** Input mesh */
    ITKMesh::Pointer mesh_;
    /** Voxel size (um) in mesh/volume space */
    double voxelSize_{100};
    /** Target density factor */
    double density_{50};
    /** Resulting number of vertices */
    std::size_t numVerts_{0};

public:
    /** @brief Input mesh */
    smgl::InputPort<ITKMesh::Pointer> mesh;
    /**
     * @brief Voxel size (um) in the mesh's coordinate space
     *
     * This is the voxel size of the Volume from which the mesh was produced.
     */
    smgl::InputPort<double> voxelSize;
    /** @brief Target density factor */
    smgl::InputPort<double> density;
    /** @brief Resulting number of vertices */
    smgl::OutputPort<std::size_t> numVerts;

    /** Constructor */
    CalculateNumVertsNode();

private:
    /** Smeagol custom serialization */
    auto serialize_(bool /*useCache*/, const filesystem::path& /*cacheDir*/)
        -> smgl::Metadata override;

    /** Smeagol custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta,
        const filesystem::path& /*cacheDir*/) override;
};

/**
 * @copybrief meshing::LaplacianSmooth
 *
 * @ingroup Graph
 */
class LaplacianSmoothMeshNode : public smgl::Node
{
private:
    /** Mesh smoothing class type */
    using Smoother = meshing::LaplacianSmooth;
    /** Mesh smoother */
    Smoother smoother_;
    /** Smoothed mesh */
    ITKMesh::Pointer mesh_;

public:
    /** @brief Input mesh */
    smgl::InputPort<ITKMesh::Pointer> input;
    /**@brief Output mesh */
    smgl::OutputPort<ITKMesh::Pointer> output;

    /** Constructor */
    LaplacianSmoothMeshNode();

private:
    /** Smeagol custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** Smeagol custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @copybrief meshing::ACVD
 *
 * @see meshing::ACVD
 * @ingroup Graph
 */
class ResampleMeshNode : public smgl::Node
{
private:
    /** Resampler class type */
    using ACVD = meshing::ACVD;
    /** Mesh resampler */
    ACVD acvd_;
    /** Output mesh */
    ITKMesh::Pointer mesh_;

public:
    /** @copydoc ACVD::Mode */
    using Mode = ACVD::Mode;

    /** @brief Input mesh */
    smgl::InputPort<ITKMesh::Pointer> input;
    /** @copydoc ACVD::setMode(Mode) */
    smgl::InputPort<Mode> mode;
    /** @copydoc ACVD::setNumberOfClusters(std::size_t) */
    smgl::InputPort<std::size_t> numVertices;
    /** @copydoc ACVD::setGradation(double) */
    smgl::InputPort<double> gradation;
    /** @copydoc ACVD::setSubsampleThreshold(std::size_t) */
    smgl::InputPort<std::size_t> subsampleThreshold;
    /** @copydoc ACVD::setQuadricsOptimizationLevel(std::size_t) */
    smgl::InputPort<std::size_t> quadricsOptimizationLevel;
    /** @brief Resampled mesh */
    smgl::OutputPort<ITKMesh::Pointer> output;

    /** Constructor */
    ResampleMeshNode();

private:
    /** Smeagol custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** Smeagol custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @copydoc meshing::UVMapToITKMesh
 *
 * @see meshing::UVMapToITKMesh
 * @ingroup Graph
 */
class UVMapToMeshNode : public smgl::Node
{
private:
    /** Converter */
    meshing::UVMapToITKMesh mesher_{};
    /** Scale the output mesh to the dims of the UVMap */
    bool scaleDims_{false};
    /** Output mesh */
    ITKMesh::Pointer output_{nullptr};

public:
    /** @brief Input mesh */
    smgl::InputPort<ITKMesh::Pointer> inputMesh;
    /** @brief Input UVMap */
    smgl::InputPort<UVMap::Pointer> uvMap;
    /** @copydoc meshing::UVMapToITKMesh::setScaleToUVDimensions(bool) */
    smgl::InputPort<bool> scaleToUVDimensions;
    /** @brief UV mesh */
    smgl::OutputPort<ITKMesh::Pointer> outputMesh;

    /** Constructor */
    UVMapToMeshNode();

private:
    /** Smeagol custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** Smeagol custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

}  // namespace volcart