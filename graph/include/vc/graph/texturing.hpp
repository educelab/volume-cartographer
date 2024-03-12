#pragma once

/** @file */

#include <cstdint>
#include <limits>

#include <opencv2/core.hpp>
#include <smgl/Node.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/neighborhood/NeighborhoodGenerator.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/UVMap.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/CompositeTexture.hpp"
#include "vc/texturing/FlatteningError.hpp"
#include "vc/texturing/IntegralTexture.hpp"
#include "vc/texturing/IntersectionTexture.hpp"
#include "vc/texturing/LayerTexture.hpp"
#include "vc/texturing/OrthographicProjectionFlattening.hpp"
#include "vc/texturing/PPMGenerator.hpp"
#include "vc/texturing/ThicknessTexture.hpp"

namespace volcart
{

/**
 * @copybrief texturing::AngleBasedFlattening
 *
 * @see texturing::AngleBasedFlattening
 * @ingroup Graph
 */
class ABFNode : public smgl::Node
{
private:
    /** Flattening class type */
    using ABF = texturing::AngleBasedFlattening;
    /** Flattening class */
    ABF abf_{};
    /** Output UV Map */
    UVMap::Pointer uvMap_{};
    /** Output flattened mesh */
    ITKMesh::Pointer mesh_{nullptr};

public:
    /** @brief Input mesh */
    smgl::InputPort<ITKMesh::Pointer> input;
    /** @copydoc ABF::setUseABF(bool) */
    smgl::InputPort<bool> useABF;
    /** @brief Flattened mesh */
    smgl::OutputPort<ITKMesh::Pointer> output;
    /** @brief UVMap generated from flattened mesh */
    smgl::OutputPort<UVMap::Pointer> uvMap;

    /** Constructor */
    ABFNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @copybrief texturing::OrthographicProjectionFlattening
 *
 * @see texturing::OrthographicProjectionFlattening
 * @ingroup Graph
 */
class OrthographicFlatteningNode : public smgl::Node
{
private:
    /** Flattening class type */
    using Ortho = texturing::OrthographicProjectionFlattening;
    /** Flattening class */
    Ortho ortho_{};
    /** Output UVMap */
    UVMap::Pointer uvMap_{};
    /** Output mesh */
    ITKMesh::Pointer mesh_{nullptr};

public:
    /** @brief Input mesh */
    smgl::InputPort<ITKMesh::Pointer> input;
    /** @brief Flattened mesh */
    smgl::OutputPort<ITKMesh::Pointer> output;
    /** @brief UVMap generated from flattened mesh */
    smgl::OutputPort<UVMap::Pointer> uvMap;

    /** Constructor */
    OrthographicFlatteningNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @copybrief texturing::LStretch()
 *
 * @see texturing::LStretch()
 * @ingroup Graph
 */
class FlatteningErrorNode : public smgl::Node
{
private:
    /** Metrics type alias */
    using Metrics = texturing::LStretchMetrics;
    /** Input 3D mesh */
    ITKMesh::Pointer mesh3D_{nullptr};
    /** Input 2D mesh */
    ITKMesh::Pointer mesh2D_{nullptr};
    /** Resulting error metrics */
    Metrics error_{};

public:
    /** @brief Input mesh (3D) */
    smgl::InputPort<ITKMesh::Pointer> mesh3D;
    /** @brief Input mesh (2D) */
    smgl::InputPort<ITKMesh::Pointer> mesh2D;
    /** @brief Calculated error metrics */
    smgl::OutputPort<Metrics> error;

    /** Constructor */
    FlatteningErrorNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool /*useCache*/, const filesystem::path& /*cacheDir*/)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta,
        const filesystem::path& /*cacheDir*/) override;
};

/**
 * @copybrief texturing::PlotLStretchError
 *
 * @see texturing::PlotLStretchError
 * @ingroup Graph
 */
class PlotLStretchErrorNode : public smgl::Node
{
private:
    /** Metrics type alias */
    using Metrics = texturing::LStretchMetrics;
    /** Error metrics */
    Metrics error_{};
    /** Per-pixel cell map */
    cv::Mat cellMap_{};
    /** Color map */
    ColorMap colorMap_{ColorMap::Plasma};
    /** Whether to add a legend to the image */
    bool drawLegend_{false};
    /** L2 error image */
    cv::Mat l2Plot_{};
    /** LInf error image */
    cv::Mat lInfPlot_{};

public:
    /** @brief Input error metrics */
    smgl::InputPort<Metrics> error;
    /** @brief Per-pixel mesh cell assignment, generally from a PerPixelMap */
    smgl::InputPort<cv::Mat> cellMap;
    /** @brief Color map/LUT for visualization */
    smgl::InputPort<ColorMap> colorMap;
    /** @brief Whether to add a legend to the image */
    smgl::InputPort<bool> drawLegend;
    /** @brief L2 error image */
    smgl::OutputPort<cv::Mat> l2Plot;
    /** @brief LInf error image */
    smgl::OutputPort<cv::Mat> lInfPlot;

    /** Constructor */
    PlotLStretchErrorNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @copybrief texturing::PPMGenerator
 *
 * @see texturing::PPMGenerator
 * @ingroup Graph
 */
class PPMGeneratorNode : public smgl::Node
{
private:
    /** Generator class type */
    using PPMGen = texturing::PPMGenerator;
    /** Generator */
    PPMGen ppmGen_;
    /** Shading method */
    PPMGen::Shading shading_{PPMGen::Shading::Smooth};
    /** Output PPM */
    PerPixelMap::Pointer ppm_;

public:
    /**
     * @copydoc PPMGen::Shading
     * @see PPMGen::Shading
     */
    using Shading = PPMGen::Shading;
    /** @brief Input mesh */
    smgl::InputPort<ITKMesh::Pointer> mesh;
    /** @brief Input UVMap */
    smgl::InputPort<UVMap::Pointer> uvMap;
    /** @brief Pixel normal shading method */
    smgl::InputPort<Shading> shading;
    /** @brief Output PerPixelMap */
    smgl::OutputPort<PerPixelMap::Pointer> ppm;

    /** Constructor */
    PPMGeneratorNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @brief Auto-calculate a texturing neighborhood's 3D radius in voxels given
 * the expected layer thickness and the texturing Volume's voxel size.
 *
 * @ingroup Graph
 */
class CalculateNeighborhoodRadiusNode : public smgl::Node
{
private:
    /** Layer thickness (um) */
    double thickness_{300};
    /** Volume voxel size */
    double voxelSize_{100};
    /** 3D radius */
    cv::Vec3d radius_;

public:
    /**
     * @brief Estimated layer thickness (um)
     * @see VolumePkg::materialThickness()
     */
    smgl::InputPort<double> thickness;
    /** @brief Texturing Volume voxel size (um) */
    smgl::InputPort<double> voxelSize;
    /** @brief 3D neighborhood radius */
    smgl::OutputPort<cv::Vec3d> radius;

    /** Constructor */
    CalculateNeighborhoodRadiusNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool /*useCache*/, const filesystem::path& /*cacheDir*/)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta,
        const filesystem::path& /*cacheDir*/) override;
};

/**
 * @brief Configure a NeighborhoodGenerator for use by a texturing algorithm
 *
 * @ingroup Graph
 */
class NeighborhoodGeneratorNode : public smgl::Node
{
public:
    /** @brief Neighborhood shape */
    enum class Shape {
        /** @see LineGenerator */
        Line = 0,
        /** @see CuboidGenerator */
        Cuboid
    };

private:
    /** Generator class type */
    using Generator = NeighborhoodGenerator;
    /** Neighborhood shape */
    Shape shape_{Shape::Line};
    /** 3D neighborhood radius */
    cv::Vec3d radius_{1, 1, 1};
    /** Sampling interval */
    double interval_{1};
    /** Sampling direction */
    Direction dir_{Direction::Bidirectional};
    /** Generator */
    Generator::Pointer gen_;

public:
    /** @brief Neighborhood shape */
    smgl::InputPort<Shape> shape;
    /** @brief 3D neighborhood radius */
    smgl::InputPort<cv::Vec3d> radius;
    /** @brief Sampling rate along the radius */
    smgl::InputPort<double> interval;
    /**
     * @brief Sampling direction
     * @see Direction
     */
    smgl::InputPort<Direction> direction;
    /** @brief Configured neighborhood generator */
    smgl::OutputPort<Generator::Pointer> generator;

    /** Constructor */
    NeighborhoodGeneratorNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool /*useCache*/, const filesystem::path& /*cacheDir*/)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta,
        const filesystem::path& /*cacheDir*/) override;
};

/**
 * @copybrief texturing::CompositeTexture
 * @see texturing::CompositeTexture
 * @ingroup Graph
 */
class CompositeTextureNode : public smgl::Node
{
private:
    /** Algorithm class type */
    using TAlgo = texturing::CompositeTexture;
    /** Generator class type */
    using Generator = NeighborhoodGenerator::Pointer;
    /** Texturing algorithm */
    TAlgo textureGen_;
    /** Composite filter */
    TAlgo::Filter filter_{TAlgo::Filter::Maximum};
    /** Output image */
    cv::Mat texture_;

public:
    /** @copydoc texturing::CompositeTexture::Filter */
    using Filter = TAlgo::Filter;

    /** @brief Input PerPixelMap */
    smgl::InputPort<PerPixelMap::Pointer> ppm;
    /** @brief Input Volume */
    smgl::InputPort<Volume::Pointer> volume;
    /** @brief Neighborhood generator */
    smgl::InputPort<Generator> generator;
    /** @brief Composite filter type */
    smgl::InputPort<Filter> filter;
    /** @brief Generated texture image */
    smgl::OutputPort<cv::Mat> texture;

    /** Constructor */
    CompositeTextureNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @copybrief texturing::IntersectionTexture
 * @see texturing::IntersectionTexture
 * @ingroup Graph
 */
class IntersectionTextureNode : public smgl::Node
{
private:
    /** Algorithm class type */
    using TAlgo = texturing::IntersectionTexture;
    /** Texturing algorithm */
    TAlgo textureGen_;
    /** Output image */
    cv::Mat texture_;

public:
    /** @brief Input PerPixelMap */
    smgl::InputPort<PerPixelMap::Pointer> ppm;
    /** @brief Input Volume */
    smgl::InputPort<Volume::Pointer> volume;
    /** @brief Generated texture image */
    smgl::OutputPort<cv::Mat> texture;

    /** Constructor */
    IntersectionTextureNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @copybrief texturing::IntegralTexture
 * @see texturing::IntegralTexture
 * @ingroup Graph
 */
class IntegralTextureNode : public smgl::Node
{
private:
    /** Algorithm class type */
    using TAlgo = texturing::IntegralTexture;
    /** Generator class type */
    using Generator = NeighborhoodGenerator::Pointer;
    /** Texturing algorithm */
    TAlgo textureGen_;
    /** Output image */
    cv::Mat texture_;

public:
    /**
     * @copybrief texturing::IntegralTexture::WeightMethod
     * @see texturing::IntegralTexture::WeightMethod
     */
    using WeightMethod = texturing::IntegralTexture::WeightMethod;
    /**
     * @copybrief texturing::IntegralTexture::LinearWeightDirection
     * @see texturing::IntegralTexture::LinearWeightDirection
     */
    using WeightDirection = texturing::IntegralTexture::LinearWeightDirection;
    /**
     * @copybrief texturing::IntegralTexture::ExpoDiffBaseMethod
     * @see texturing::IntegralTexture::ExpoDiffBaseMethod
     */
    using ExpoDiffBaseMethod = texturing::IntegralTexture::ExpoDiffBaseMethod;

    /** @brief Input PerPixelMap */
    smgl::InputPort<PerPixelMap::Pointer> ppm;
    /** @brief Input Volume */
    smgl::InputPort<Volume::Pointer> volume;
    /** @brief Neighborhood generator */
    smgl::InputPort<Generator> generator;
    /** @copybrief texturing::IntegralTexture::setClampValuesToMax() */
    smgl::InputPort<bool> clampValuesToMax;
    /** @copybrief texturing::IntegralTexture::setClampMax() */
    smgl::InputPort<std::uint16_t> clampMax;
    /** @copybrief texturing::IntegralTexture::setWeightMethod() */
    smgl::InputPort<WeightMethod> weightMethod;
    /** @copybrief texturing::IntegralTexture::setLinearWeightDirection() */
    smgl::InputPort<WeightDirection> linearWeightDirection;
    /** @copybrief texturing::IntegralTexture::setExponentialDiffExponent() */
    smgl::InputPort<int> exponentialDiffExponent;
    /** @copybrief texturing::IntegralTexture::setExponentialDiffBaseMethod() */
    smgl::InputPort<ExpoDiffBaseMethod> exponentialDiffBaseMethod;
    /** @copybrief texturing::IntegralTexture::setExponentialDiffBaseValue() */
    smgl::InputPort<double> exponentialDiffBaseValue;
    /** @copybrief TAlgo::setExponentialDiffSuppressBelowBase() */
    smgl::InputPort<bool> exponentialDiffSuppressBelowBase;
    /** @brief Generated texture image */
    smgl::OutputPort<cv::Mat> texture;

    /** Constructor */
    IntegralTextureNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @copybrief texturing::ThicknessTexture
 * @see texturing::ThicknessTexture
 * @ingroup Graph
 */
class ThicknessTextureNode : public smgl::Node
{
private:
    /** Algorithm class type */
    using TAlgo = texturing::ThicknessTexture;
    /** Texturing algorithm */
    TAlgo textureGen_;
    /** Output image */
    cv::Mat texture_;

public:
    /** @brief Input PerPixelMap */
    smgl::InputPort<PerPixelMap::Pointer> ppm;
    /** @brief Input Volume */
    smgl::InputPort<Volume::Pointer> volume;
    /** @brief VolumetricMask for a single layer */
    smgl::InputPort<VolumetricMask::Pointer> volumetricMask;
    /** @copybrief texturing::ThicknessTexture::setSamplingInterval() */
    smgl::InputPort<double> samplingInterval;
    /** @copybrief texturing::ThicknessTexture::setNormalizeOutput() */
    smgl::InputPort<bool> normalizeOutput;
    /** @brief Generated texture image */
    smgl::OutputPort<cv::Mat> texture;

    /** Constructor */
    ThicknessTextureNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};

/**
 * @copybrief texturing::LayerTexture
 * @see texturing::LayerTexture
 * @ingroup Graph
 */
class LayerTextureNode : public smgl::Node
{
private:
    /** Image list type */
    using ImageList = std::vector<cv::Mat>;
    /** Algorithm class type */
    using TAlgo = texturing::LayerTexture;
    /** Generator class type */
    using Generator = NeighborhoodGenerator::Pointer;
    /** Texturing algorithm */
    TAlgo textureGen_;
    /** Output layer images */
    ImageList texture_;

public:
    /** @brief Input PerPixelMap */
    smgl::InputPort<PerPixelMap::Pointer> ppm;
    /** @brief Input Volume */
    smgl::InputPort<Volume::Pointer> volume;
    /**
     * @brief Neighborhood generator
     *
     * @throws std::runtime_error If the provided generator is not a
     * LineGenerator.
     */
    smgl::InputPort<Generator> generator;
    /** @brief Generated texture image */
    smgl::OutputPort<ImageList> texture;

    /** Constructor */
    LayerTextureNode();

private:
    /** smgl custom serialization */
    auto serialize_(bool useCache, const filesystem::path& cacheDir)
        -> smgl::Metadata override;

    /** smgl custom deserialization */
    void deserialize_(
        const smgl::Metadata& meta, const filesystem::path& cacheDir) override;
};
}  // namespace volcart