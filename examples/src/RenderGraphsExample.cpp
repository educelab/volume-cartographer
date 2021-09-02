#include <smgl/Graphviz.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/graph.hpp"

using namespace smgl;
using namespace volcart;

namespace fs = volcart::filesystem;

auto main() -> int
{
    // Register all nodes with the factory
    RegisterNodes();

    // Setup output dir
    fs::path outDir = "En-Gedi results/";
    fs::create_directories(outDir);

    // Setup a graph
    Graph g;
    g.setEnableCache(true);
    g.setCacheFile(outDir / "EnGedi_graph.json");
    g.setCacheType(CacheType::Subdirectory);

    // Cleanup old cache dir
    fs::remove_all(g.cacheDir());

    // Load the volumepkg
    auto vpkg = g.insertNode<LoadVolumePkgNode>();
    vpkg->path =
        "/Volumes/201503-VC/VolPkgs/EinGediFull-OffsetScan-2015.volpkg/";

    // Package props
    auto vpkgProps = g.insertNode<VolumePkgPropertiesNode>();
    vpkgProps->volpkg = vpkg->volpkg;

    // Load the volume
    auto vol = g.insertNode<VolumeSelectorNode>();
    vol->volpkg = vpkg->volpkg;

    // Load the segmentation
    auto seg = g.insertNode<SegmentationSelectorNode>();
    seg->id = "20160105173647";
    seg->volpkg = vpkg->volpkg;

    // Get the segmentation pointset
    auto segProps = g.insertNode<SegmentationPropertiesNode>();
    segProps->segmentation = seg->segmentation;

    // Mesh the pointset
    auto mesher = g.insertNode<MeshingNode>();
    mesher->points = segProps->pointSet;

    // Volume properties
    auto volProps = g.insertNode<VolumePropertiesNode>();
    volProps->volumeIn = vol->volume;

    // Calculate new density
    auto calcVerts = g.insertNode<CalculateNumVertsNode>();
    calcVerts->mesh = mesher->mesh;
    calcVerts->voxelSize = volProps->voxelSize;

    // Resample the mesh
    auto resample = g.insertNode<ResampleMeshNode>();
    resample->input = mesher->mesh;
    resample->numVertices = calcVerts->numVerts;

    // Flatten the mesh
    auto flatten = g.insertNode<ABFNode>();
    flatten->input = resample->output;

    // Rotate the UV map
    auto rotUV = g.insertNode<RotateUVMapNode>();
    rotUV->uvMapIn = flatten->uvMap;
    rotUV->theta = -98.7;

    // Generate PPM
    auto ppmGen = g.insertNode<PPMGeneratorNode>();
    ppmGen->mesh = resample->output;
    ppmGen->uvMap = rotUV->uvMapOut;

    // Auto-calc radius
    auto radiusCalc = g.insertNode<CalculateNeighborhoodRadiusNode>();
    radiusCalc->thickness = vpkgProps->materialThickness;
    radiusCalc->voxelSize = volProps->voxelSize;

    // Neighborhood generator
    auto neighborGen = g.insertNode<NeighborhoodGeneratorNode>();
    neighborGen->radius = radiusCalc->radius;

    // Texture #1
    auto texturing = g.insertNode<CompositeTextureNode>();
    texturing->filter(CompositeTextureNode::Filter::Maximum);
    texturing->ppm = ppmGen->ppm;
    texturing->volume = vol->volume;
    texturing->generator = neighborGen->generator;

    // Write mesh #1
    auto writer = g.insertNode<WriteMeshNode>();
    writer->path(outDir / "EnGedi_Result_MaxMesh.obj");
    writer->mesh = resample->output;
    writer->uvMap = rotUV->uvMapOut;
    writer->texture = texturing->texture;

    // Update the graph
    g.update();
    WriteDotFile(outDir / "EnGedi_graph.gv", g);
}
