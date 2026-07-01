#include "vc/graph.hpp"

#include <smgl/Node.hpp>

using namespace volcart;

namespace
{
auto RegisterNodesImpl() -> bool
{
    bool registered{true};

    // clang-format off
    // Core
    registered &= smgl::RegisterNodes(
        SMGL_NODE(volcart::LoadVolumePkgNode),
        SMGL_NODE(volcart::VolumePkgPropertiesNode),
        SMGL_NODE(volcart::VolumeSelectorNode),
        SMGL_NODE(volcart::VolumePropertiesNode),
        SMGL_NODE(volcart::SegmentationSelectorNode),
        SMGL_NODE(volcart::SegmentationPropertiesNode),
        SMGL_NODE(volcart::MeshPropertiesNode),
        SMGL_NODE(volcart::LoadMeshNode),
        SMGL_NODE(volcart::WriteMeshNode),
        SMGL_NODE(volcart::AlignUVMapToAxisNode),
        SMGL_NODE(volcart::RotateUVMapNode),
        SMGL_NODE(volcart::FlipUVMapNode),
        SMGL_NODE(volcart::PlotUVMapNode),
        SMGL_NODE(volcart::LoadImageNode),
        SMGL_NODE(volcart::WriteImageNode),
        SMGL_NODE(volcart::WriteImageSequenceNode),
        SMGL_NODE(volcart::LoadPPMNode),
        SMGL_NODE(volcart::WritePPMNode),
        SMGL_NODE(volcart::PPMPropertiesNode),
        SMGL_NODE(volcart::LoadVolumetricMaskNode),
        SMGL_NODE(volcart::LoadTransformNode),
        SMGL_NODE(volcart::TransformSelectorNode),
        SMGL_NODE(volcart::InvertTransformNode),
        SMGL_NODE(volcart::TransformMeshNode),
        SMGL_NODE(volcart::TransformPPMNode));

    // Meshing
    registered &= smgl::RegisterNodes(
        SMGL_NODE(volcart::MeshingNode),
        SMGL_NODE(volcart::ScaleMeshNode),
        SMGL_NODE(volcart::CalculateNumVertsNode),
        SMGL_NODE(volcart::LaplacianSmoothMeshNode),
        SMGL_NODE(volcart::ResampleMeshNode),
        SMGL_NODE(volcart::UVMapToMeshNode),
        SMGL_NODE(volcart::OrientNormalsNode));

    // Texturing
    registered &= smgl::RegisterNodes(
        SMGL_NODE(volcart::ABFNode),
        SMGL_NODE(volcart::OrthographicFlatteningNode),
        SMGL_NODE(volcart::FlatteningErrorNode),
        SMGL_NODE(volcart::PlotLStretchErrorNode),
        SMGL_NODE(volcart::PPMGeneratorNode),
        SMGL_NODE(volcart::CalculateNeighborhoodRadiusNode),
        SMGL_NODE(volcart::NeighborhoodGeneratorNode),
        SMGL_NODE(volcart::CompositeTextureNode),
        SMGL_NODE(volcart::IntersectionTextureNode),
        SMGL_NODE(volcart::IntegralTextureNode),
        SMGL_NODE(volcart::ThicknessTextureNode),
        SMGL_NODE(volcart::LayerTextureNode));
    // clang-format on

    return registered;
}
}  // namespace

void volcart::RegisterNodes()
{
    static auto registered = ::RegisterNodesImpl();
}
