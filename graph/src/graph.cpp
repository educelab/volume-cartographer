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
    registered &= smgl::RegisterNode<
        LoadVolumePkgNode,
        VolumePkgPropertiesNode,
        VolumeSelectorNode,
        VolumePropertiesNode,
        SegmentationSelectorNode,
        SegmentationPropertiesNode,
        MeshPropertiesNode,
        LoadMeshNode,
        WriteMeshNode,
        AlignUVMapToAxisNode,
        RotateUVMapNode,
        FlipUVMapNode,
        PlotUVMapNode,
        LoadImageNode,
        WriteImageNode,
        LoadPPMNode,
        WritePPMNode,
        PPMPropertiesNode,
        LoadVolumetricMaskNode,
        LoadTransformNode,
        TransformSelectorNode,
        InvertTransformNode,
        TransformMeshNode,
        TransformPPMNode
    >();

    // Meshing
    registered &= smgl::RegisterNode<
        MeshingNode,
        ScaleMeshNode,
        CalculateNumVertsNode,
        LaplacianSmoothMeshNode,
        ResampleMeshNode,
        UVMapToMeshNode,
        OrientNormalsNode
    >();

    // Texturing
    registered &= smgl::RegisterNode<
        ABFNode,
        OrthographicFlatteningNode,
        FlatteningErrorNode,
        PlotLStretchErrorNode,
        PPMGeneratorNode,
        CalculateNeighborhoodRadiusNode,
        NeighborhoodGeneratorNode,
        CompositeTextureNode,
        IntersectionTextureNode,
        IntegralTextureNode,
        ThicknessTextureNode
    >();
    // clang-format on

    return registered;
}
}  // namespace

void volcart::RegisterNodes()
{
    static auto registered = ::RegisterNodesImpl();
}
