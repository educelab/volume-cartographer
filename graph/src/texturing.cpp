#include "vc/graph/texturing.hpp"

#include <nlohmann/json.hpp>

#include "vc/core/io/ImageIO.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/io/UVMapIO.hpp"
#include "vc/core/neighborhood/CuboidGenerator.hpp"
#include "vc/core/neighborhood/LineGenerator.hpp"
#include "vc/core/types/PointSet.hpp"
#include "vc/core/util/FloatComparison.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
using namespace volcart::texturing;
namespace fs = volcart::filesystem;

// Enum conversions
namespace volcart
{
// clang-format off
using Shape = NeighborhoodGeneratorNode::Shape;
NLOHMANN_JSON_SERIALIZE_ENUM(Shape, {
    {Shape::Line, "line"},
    {Shape::Cuboid, "cuboid"},
})

NLOHMANN_JSON_SERIALIZE_ENUM(Direction, {
    {Direction::Negative, "negative"},
    {Direction::Bidirectional, "bidirectional"},
    {Direction::Positive, "positive"},
})
// clang-format on
}  // namespace volcart

namespace volcart::texturing
{
// clang-format off
using Shading = PPMGeneratorNode::Shading;
NLOHMANN_JSON_SERIALIZE_ENUM(Shading, {
    {Shading::Flat, "flat"},
    {Shading::Smooth, "smooth"}
})

using Filter = CompositeTextureNode::Filter;
NLOHMANN_JSON_SERIALIZE_ENUM(Filter, {
    {Filter::Minimum, "minimum"},
    {Filter::Maximum, "maximum"},
    {Filter::Median, "median"},
    {Filter::Mean, "mean"},
    {Filter::MedianAverage, "median_average"}
})

using WeightMethod = IntegralTextureNode::WeightMethod;
NLOHMANN_JSON_SERIALIZE_ENUM(WeightMethod, {
    {WeightMethod::None, "none"},
    {WeightMethod::Linear, "linear"},
    {WeightMethod::ExpoDiff, "expodiff"},
})

using WeightDirection = IntegralTextureNode::WeightDirection;
NLOHMANN_JSON_SERIALIZE_ENUM(WeightDirection, {
    {WeightDirection::Positive, "positive"},
    {WeightDirection::Negative, "negative"},
})

using ExpoDiffBaseMethod = IntegralTextureNode::ExpoDiffBaseMethod;
NLOHMANN_JSON_SERIALIZE_ENUM(ExpoDiffBaseMethod, {
    {ExpoDiffBaseMethod::Mean, "mean"},
    {ExpoDiffBaseMethod::Mode, "mode"},
    {ExpoDiffBaseMethod::Manual, "manual"}
})
// clang-format on
}  // namespace volcart::texturing

ABFNode::ABFNode()
    : Node{true}
    , input{&abf_, &ABF::setMesh}
    , useABF{&abf_, &ABF::setUseABF}
    , output{&mesh_}
    , uvMap{&uvMap_}
{
    registerInputPort("input", input);
    registerInputPort("useABF", useABF);
    registerOutputPort("output", output);
    registerOutputPort("uvMap", uvMap);

    compute = [&]() {
        mesh_ = abf_.compute();
        uvMap_ = abf_.getUVMap();
    };
}

auto ABFNode::serialize_(bool useCache, const fs::path& cacheDir)
    -> smgl::Metadata
{
    smgl::Metadata meta{
        {"useABF", abf_.useABF()},
        {"abfMaxIterations", abf_.abfMaxIterations()}};

    if (useCache and uvMap_ and not uvMap_->empty()) {
        io::WriteUVMap(cacheDir / "uvMap.uvm", *uvMap_);
        meta["uvMap"] = "uvMap.uvm";
        WriteMesh(cacheDir / "uvMesh.obj", mesh_);
        meta["mesh"] = "uvMesh.obj";
    }
    return meta;
}

void ABFNode::deserialize_(const smgl::Metadata& meta, const fs::path& cacheDir)
{
    abf_.setUseABF(meta["useABF"].get<bool>());
    abf_.setABFMaxIterations(meta["abfMaxIterations"].get<std::size_t>());

    if (meta.contains("uvMap")) {
        auto file = meta["uvMap"].get<std::string>();
        uvMap_ = UVMap::New(io::ReadUVMap(cacheDir / file));
    }

    if (meta.contains("uvMesh")) {
        auto file = meta["uvMesh"].get<std::string>();
        mesh_ = ReadMesh(cacheDir / file).mesh;
    }
}

OrthographicFlatteningNode::OrthographicFlatteningNode()
    : Node{true}
    , input{&ortho_, &Ortho::setMesh}
    , output{&mesh_}
    , uvMap{&uvMap_}
{
    registerInputPort("input", input);
    registerOutputPort("output", output);
    registerOutputPort("uvMap", uvMap);

    compute = [&]() {
        mesh_ = ortho_.compute();
        uvMap_ = ortho_.getUVMap();
    };
}

auto OrthographicFlatteningNode::serialize_(
    bool useCache, const fs::path& cacheDir) -> smgl::Metadata
{
    smgl::Metadata meta;
    if (useCache and uvMap_ and not uvMap_->empty()) {
        io::WriteUVMap(cacheDir / "uvMap.uvm", *uvMap_);
        meta["uvMap"] = "uvMap.uvm";
        WriteMesh(cacheDir / "uvMesh.obj", mesh_);
        meta["mesh"] = "uvMesh.obj";
    }
    return meta;
}

void OrthographicFlatteningNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    if (meta.contains("uvMap")) {
        auto file = meta["uvMap"].get<std::string>();
        uvMap_ = UVMap::New(io::ReadUVMap(cacheDir / file));
    }

    if (meta.contains("uvMesh")) {
        auto file = meta["uvMesh"].get<std::string>();
        mesh_ = ReadMesh(cacheDir / file).mesh;
    }
}

FlatteningErrorNode::FlatteningErrorNode()
    : mesh3D{&mesh3D_}, mesh2D{&mesh2D_}, error{&error_}
{
    registerInputPort("mesh3D", mesh3D);
    registerInputPort("mesh2D", mesh2D);
    registerOutputPort("error", error);

    compute = [&]() {
        if (mesh3D_ and mesh2D_) {
            error_ = LStretch(mesh3D_, mesh2D_);
            Logger()->info(
                "L2 Norm: {:.5g}, LInf Norm: {:.5g}", error_.l2, error_.lInf);
        }
    };
}

auto FlatteningErrorNode::serialize_(
    bool /*useCache*/, const filesystem::path& /*cacheDir*/) -> smgl::Metadata
{
    smgl::Metadata meta;
    if (mesh3D_ and mesh2D_) {
        meta["l2"] = error_.l2;
        meta["lInf"] = error_.lInf;
        meta["faceL2"] = error_.faceL2;
        meta["faceLInf"] = error_.faceLInf;
    }
    return meta;
}

void FlatteningErrorNode::deserialize_(
    const smgl::Metadata& meta, const filesystem::path& /*cacheDir*/)
{
    if (meta.empty()) {
        return;
    }

    error_.l2 = meta["l2"].get<double>();
    error_.lInf = meta["lInf"].get<double>();
    error_.faceL2 = meta["faceL2"].get<std::vector<double>>();
    error_.faceLInf = meta["faceLInf"].get<std::vector<double>>();
}

PlotLStretchErrorNode::PlotLStretchErrorNode()
    : Node{true}
    , error{&error_}
    , cellMap{&cellMap_}
    , colorMap{&colorMap_}
    , drawLegend{&drawLegend_}
    , l2Plot{&l2Plot_}
    , lInfPlot{&lInfPlot_}
{
    registerInputPort("error", error);
    registerInputPort("cellMap", cellMap);
    registerInputPort("colorMap", colorMap);
    registerInputPort("drawLegend", drawLegend);
    registerOutputPort("l2Plot", l2Plot);
    registerOutputPort("lInfPlot", lInfPlot);

    compute = [&]() {
        auto p = PlotLStretchError(error_, cellMap_, colorMap_, drawLegend_);
        l2Plot_ = p[0];
        lInfPlot_ = p[1];
    };
}

auto PlotLStretchErrorNode::serialize_(
    bool useCache, const filesystem::path& cacheDir) -> smgl::Metadata
{
    smgl::Metadata meta{
        {"colorMap", ColorMapToString(colorMap_)}, {"drawLegend", drawLegend_}};
    if (useCache) {
        if (not l2Plot_.empty()) {
            WriteImage(cacheDir / "l2Plot.tif", l2Plot_);
            meta["l2Plot"] = "l2Plot.tif";
        }
        if (not lInfPlot_.empty()) {
            WriteImage(cacheDir / "lInfPlot.tif", lInfPlot_);
            meta["lInfPlot"] = "lInfPlot.tif";
        }
    }
    return meta;
}

void PlotLStretchErrorNode::deserialize_(
    const smgl::Metadata& meta, const filesystem::path& cacheDir)
{
    colorMap_ = ColorMapFromString(meta["colorMap"].get<std::string>());
    drawLegend_ = meta["drawLegend"].get<bool>();
    if (meta.contains("l2Plot")) {
        l2Plot_ = ReadImage(cacheDir / meta["l2Plot"].get<std::string>());
    }
    if (meta.contains("lInfPlot")) {
        lInfPlot_ = ReadImage(cacheDir / meta["lInfPlot"].get<std::string>());
    }
}

PPMGeneratorNode::PPMGeneratorNode()
    : Node{true}
    , mesh{&ppmGen_, &PPMGen::setMesh}
    , uvMap{[&](const auto& uv) {
        auto width = static_cast<size_t>(std::ceil(uv->ratio().width));
        auto height = static_cast<size_t>(std::ceil(uv->ratio().height));
        ppmGen_.setUVMap(uv);
        ppmGen_.setDimensions(height, width);
    }}
    , shading{[&](const auto& s) {
        shading_ = s;
        ppmGen_.setShading(s);
    }}
    , ppm{&ppm_}
{
    registerInputPort("mesh", mesh);
    registerInputPort("uvMap", uvMap);
    registerInputPort("shading", shading);
    registerOutputPort("ppm", ppm);
    compute = [&]() { ppm_ = ppmGen_.compute(); };
}

auto PPMGeneratorNode::serialize_(bool useCache, const fs::path& cacheDir)
    -> smgl::Metadata
{
    smgl::Metadata meta{{"shading", shading_}};
    if (useCache and ppm_ and ppm_->initialized()) {
        PerPixelMap::WritePPM(cacheDir / "PerPixelMap.ppm", *ppm_);
        meta["ppm"] = "PerPixelMap.ppm";
    }
    return meta;
}

void PPMGeneratorNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    shading_ = meta["shading"].get<Shading>();
    if (meta.contains("ppm")) {
        auto ppmFile = meta["ppm"].get<std::string>();
        ppm_ = PerPixelMap::New(PerPixelMap::ReadPPM(cacheDir / ppmFile));
    }
}

CalculateNeighborhoodRadiusNode::CalculateNeighborhoodRadiusNode()
    : thickness{&thickness_}, voxelSize{&voxelSize_}, radius{&radius_}
{
    registerInputPort("thickness", thickness);
    registerInputPort("voxelSize", voxelSize);
    registerOutputPort("radius", radius);

    compute = [&]() {
        radius_[0] = thickness_ / 2 / voxelSize_;
        radius_[1] = radius_[2] = std::abs(std::sqrt(radius_[0]));
    };
}

auto CalculateNeighborhoodRadiusNode::serialize_(
    bool /*useCache*/, const fs::path& /*cacheDir*/) -> smgl::Metadata
{
    return {{"radius", {radius_[0], radius_[1], radius_[2]}}};
}

void CalculateNeighborhoodRadiusNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& /*cacheDir*/)
{
    std::vector<double> r;
    meta["radius"].get_to(r);
    radius_[0] = r[0];
    radius_[1] = r[1];
    radius_[1] = r[2];
}

NeighborhoodGeneratorNode::NeighborhoodGeneratorNode()
    : shape{&shape_}
    , radius{&radius_}
    , interval{&interval_}
    , direction{&dir_}
    , generator{&gen_}
{
    registerInputPort("shape", shape);
    registerInputPort("radius", radius);
    registerInputPort("interval", interval);
    registerInputPort("direction", direction);
    registerOutputPort("generator", generator);

    compute = [&]() {
        // TODO: Make a new one with every compute?
        if (shape_ == Shape::Line) {
            gen_ = LineGenerator::New();
        } else if (shape_ == Shape::Cuboid) {
            gen_ = CuboidGenerator::New();
        }
        gen_->setSamplingRadius(radius_);
        gen_->setSamplingInterval(interval_);
        gen_->setSamplingDirection(dir_);
    };
}

auto NeighborhoodGeneratorNode::serialize_(
    bool /*useCache*/, const fs::path& /*cacheDir*/) -> smgl::Metadata
{
    smgl::Metadata meta;
    meta["shape"] = shape_;
    meta["radius"] = {radius_[0], radius_[1], radius_[2]};
    meta["interval"] = interval_;
    meta["direction"] = dir_;
    return meta;
}

void NeighborhoodGeneratorNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& /*cacheDir*/)
{
    shape_ = meta["shape"].get<Shape>();

    std::vector<double> r;
    meta["radius"].get_to(r);
    radius_[0] = r[0];
    radius_[1] = r[1];
    radius_[2] = r[2];

    interval_ = meta["interval"].get<double>();
    dir_ = meta["direction"].get<Direction>();
}

CompositeTextureNode::CompositeTextureNode()
    : Node{true}
    , ppm{&textureGen_, &TAlgo::setPerPixelMap}
    , volume{&textureGen_, &TAlgo::setVolume}
    , generator{&textureGen_, &TAlgo::setGenerator}
    , filter{[&](const auto& f) {
        filter_ = f;
        textureGen_.setFilter(filter_);
    }}
    , texture{&texture_}
{
    registerInputPort("ppm", ppm);
    registerInputPort("volume", volume);
    registerInputPort("generator", generator);
    registerInputPort("filter", filter);
    registerOutputPort("texture", texture);
    compute = [&]() { texture_ = textureGen_.compute().at(0); };
}

auto CompositeTextureNode::serialize_(bool useCache, const fs::path& cacheDir)
    -> smgl::Metadata
{
    smgl::Metadata meta;
    meta["filter"] = filter_;
    if (useCache and not texture_.empty()) {
        WriteImage(cacheDir / "composite.tif", texture_);
        meta["texture"] = "composite.tif";
    }
    return meta;
}

void CompositeTextureNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    filter_ = meta["filter"].get<Filter>();
    textureGen_.setFilter(filter_);
    if (meta.contains("texture")) {
        auto imgFile = meta["texture"].get<std::string>();
        texture_ = ReadImage(cacheDir / imgFile);
    }
}

IntersectionTextureNode::IntersectionTextureNode()
    : Node{true}
    , ppm{&textureGen_, &TAlgo::setPerPixelMap}
    , volume{&textureGen_, &TAlgo::setVolume}
    , texture{&texture_}
{
    registerInputPort("ppm", ppm);
    registerInputPort("volume", volume);
    registerOutputPort("texture", texture);
    compute = [&]() { texture_ = textureGen_.compute().at(0); };
}

auto IntersectionTextureNode::serialize_(
    bool useCache, const fs::path& cacheDir) -> smgl::Metadata
{
    smgl::Metadata meta;
    if (useCache and not texture_.empty()) {
        WriteImage(cacheDir / "intersection.tif", texture_);
        meta["texture"] = "intersection.tif";
    }
    return meta;
}

void IntersectionTextureNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    if (meta.contains("texture")) {
        auto imgFile = meta["texture"].get<std::string>();
        texture_ = ReadImage(cacheDir / imgFile);
    }
}

IntegralTextureNode::IntegralTextureNode()
    : Node{true}
    , ppm{&textureGen_, &TAlgo::setPerPixelMap}
    , volume{&textureGen_, &TAlgo::setVolume}
    , generator{&textureGen_, &TAlgo::setGenerator}
    , clampValuesToMax{&textureGen_, &TAlgo::setClampValuesToMax}
    , clampMax{&textureGen_, &TAlgo::setClampMax}
    , weightMethod{&textureGen_, &TAlgo::setWeightMethod}
    , linearWeightDirection{&textureGen_, &TAlgo::setLinearWeightDirection}
    , exponentialDiffExponent{&textureGen_, &TAlgo::setExponentialDiffExponent}
    , exponentialDiffBaseMethod{&textureGen_, &TAlgo::setExponentialDiffBaseMethod}
    , exponentialDiffBaseValue{&textureGen_, &TAlgo::setExponentialDiffBaseValue}
    , exponentialDiffSuppressBelowBase{&textureGen_, &TAlgo::setExponentialDiffSuppressBelowBase}
    , texture{&texture_}
{
    registerInputPort("ppm", ppm);
    registerInputPort("volume", volume);
    registerInputPort("generator", generator);
    registerInputPort("clampValuesToMax", clampValuesToMax);
    registerInputPort("clampMax", clampMax);
    registerInputPort("weightMethod", weightMethod);
    registerInputPort("linearWeightDirection", linearWeightDirection);
    registerInputPort("exponentialDiffExponent", exponentialDiffExponent);
    registerInputPort("exponentialDiffBaseMethod", exponentialDiffBaseMethod);
    registerInputPort("exponentialDiffBaseValue", exponentialDiffBaseValue);
    registerInputPort(
        "exponentialDiffSuppressBelowBase", exponentialDiffSuppressBelowBase);
    registerOutputPort("texture", texture);

    compute = [&]() { texture_ = textureGen_.compute().at(0); };
}

auto IntegralTextureNode::serialize_(bool useCache, const fs::path& cacheDir)
    -> smgl::Metadata
{
    smgl::Metadata meta{
        {"clampToMax", textureGen_.clampValuesToMax()},
        {"clampMax", textureGen_.clampMax()},
        {"weightMethod", textureGen_.weightMethod()},
        {"linearWeightDirection", textureGen_.linearWeightDirection()},
        {"diffExponent", textureGen_.exponentialDiffExponent()},
        {"diffMethod", textureGen_.exponentialDiffBaseMethod()},
        {"diffBaseVal", textureGen_.exponentialDiffBaseValue()},
        {"diffSuppressBelowBase",
         textureGen_.exponentialDiffSuppressBelowBase()}};
    if (useCache and not texture_.empty()) {
        WriteImage(cacheDir / "integral.tif", texture_);
        meta["texture"] = "integral.tif";
    }
    return meta;
}

void IntegralTextureNode::deserialize_(
    const smgl::Metadata& meta, const fs::path& cacheDir)
{
    textureGen_.setClampValuesToMax(meta["clampToMax"].get<bool>());
    textureGen_.setClampMax(meta["clampMax"].get<uint16_t>());
    textureGen_.setWeightMethod(meta["weightMethod"].get<WeightMethod>());
    textureGen_.setLinearWeightDirection(
        meta["linearWeightDirection"].get<WeightDirection>());
    textureGen_.setExponentialDiffExponent(meta["diffExponent"].get<int>());
    textureGen_.setExponentialDiffBaseMethod(
        meta["diffMethod"].get<ExpoDiffBaseMethod>());
    textureGen_.setExponentialDiffBaseValue(meta["diffBaseVal"].get<double>());
    textureGen_.setExponentialDiffSuppressBelowBase(
        meta["diffSuppressBelowBase"].get<bool>());
    if (meta.contains("texture")) {
        auto imgFile = meta["texture"].get<std::string>();
        texture_ = ReadImage(cacheDir / imgFile);
    }
}

ThicknessTextureNode::ThicknessTextureNode()
    : Node{true}
    , ppm{&textureGen_, &TAlgo::setPerPixelMap}
    , volume{&textureGen_, &TAlgo::setVolume}
    , volumetricMask{&textureGen_, &TAlgo::setVolumetricMask}
    , samplingInterval{&textureGen_, &TAlgo::setSamplingInterval}
    , normalizeOutput{&textureGen_, &TAlgo::setNormalizeOutput}
    , texture{&texture_}
{
    registerInputPort("ppm", ppm);
    registerInputPort("volume", volume);
    registerInputPort("volumetricMask", volumetricMask);
    registerInputPort("samplingInterval", samplingInterval);
    registerInputPort("normalizeOutput", normalizeOutput);
    registerOutputPort("texture", texture);

    compute = [&]() { texture_ = textureGen_.compute().at(0); };
}

auto ThicknessTextureNode::serialize_(
    bool useCache, const filesystem::path& cacheDir) -> smgl::Metadata
{
    smgl::Metadata meta{
        {"samplingInterval", textureGen_.samplingInterval()},
        {"normalizeOutput", textureGen_.normalizeOutput()},
    };
    if (useCache) {
        if (not texture_.empty()) {
            WriteImage(cacheDir / "thickness.tif", texture_);
            meta["texture"] = "thickness.tif";
        }
    }

    return meta;
}

void ThicknessTextureNode::deserialize_(
    const smgl::Metadata& meta, const filesystem::path& cacheDir)
{
    textureGen_.setSamplingInterval(meta["samplingInterval"].get<double>());
    textureGen_.setNormalizeOutput(meta["normalizeOutput"].get<bool>());
    if (meta.contains("texture")) {
        auto imgFile = meta["texture"].get<std::string>();
        texture_ = ReadImage(cacheDir / imgFile);
    }
}
