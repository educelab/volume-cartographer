#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/neighborhood/LineGenerator.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"
#include "vc/texturing/CompositeTexture.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace vc = volcart;

int main(int /*argc*/, char* argv[])
{
    vc::VolumePkg vpkg(argv[1]);
    auto seg = vpkg.segmentation(argv[2]);

    // try to convert the ply to an ITK mesh
    vc::meshing::OrderedPointSetMesher mesher;
    mesher.setPointSet(seg->getPointSet());
    auto inputMesh = mesher.compute();

    size_t width = 608 * 2;
    size_t height = 370 * 2;

    vc::UVMap uvMap;
    uvMap.set(0, cv::Vec2d(0, 0));
    uvMap.set(1, cv::Vec2d(1, 0));
    uvMap.set(2, cv::Vec2d(0, 1));
    uvMap.set(3, cv::Vec2d(1, 1));
    uvMap.ratio(width, height);

    vc::texturing::PPMGenerator ppmGen(height, width);
    ppmGen.setMesh(inputMesh);
    ppmGen.setUVMap(uvMap);
    ppmGen.compute();

    auto generator = vc::LineGenerator::New();
    generator->setSamplingRadius(1);

    vc::texturing::CompositeTexture compText;
    compText.setPerPixelMap(ppmGen.getPPM());
    compText.setVolume(vpkg.volume());
    compText.setFilter(volcart::texturing::CompositeTexture::Filter::Minimum);
    compText.setGenerator(generator);
    compText.compute();

    vc::io::OBJWriter mesh_writer;
    mesh_writer.setPath("compV2Test.obj");
    mesh_writer.setMesh(inputMesh);
    mesh_writer.setTexture(compText.getTexture().image(0));
    mesh_writer.setUVMap(compText.getTexture().uvMap());
    mesh_writer.write();

    return EXIT_SUCCESS;
}
