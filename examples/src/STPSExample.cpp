#include <vc/core/io/PLYWriter.hpp>
#include <vc/core/types/VolumePkg.hpp>
#include <vc/meshing/OrderedPointSetMesher.hpp>
#include <vc/segmentation/StructureTensorParticleSim.hpp>

namespace vc = volcart;
namespace vcm = volcart::meshing;
namespace vcs = volcart::segmentation;

int main(int /*argc*/, char* argv[])
{
    std::string volpkgPath = argv[1];
    std::string segId = argv[2];

    vc::VolumePkg volpkg(volpkgPath);

    // Get starting chain
    auto seg = volpkg.segmentation(segId);
    auto chain = seg->getPointSet().getRow(0);

    vcs::StructureTensorParticleSim segmenter;
    segmenter.setChain(chain);
    segmenter.setVolume(volpkg.volume());
    segmenter.setNumberOfSteps(40);
    segmenter.setMaterialThickness(volpkg.getMaterialThickness());
    auto result = segmenter.compute();

    seg->setPointSet(result);

    vcm::OrderedPointSetMesher mesher;
    mesher.setPointSet(result);
    auto mesh = mesher.compute();

    vc::io::PLYWriter writer;
    writer.setPath("stps.ply");
    writer.setMesh(mesh);
    writer.write();
}