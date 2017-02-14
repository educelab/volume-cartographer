#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/PointSet.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"

namespace fs = boost::filesystem;
namespace vc = volcart;

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << argv[0] << "orderedFile.vcps" << std::endl;
        return (1);
    }

    // Load the file
    auto inputCloud = vc::PointSetIO<cv::Vec3d>::ReadOrderedPointSet(argv[1]);

    // Set the output
    fs::path outfile{argv[1]};
    outfile.replace_extension("ply");

    vc::meshing::OrderedPointSetMesher mesh(inputCloud);
    mesh.compute();

    auto output = mesh.getOutputMesh();
    vc::io::PLYWriter writer(outfile, output);

    exit(EXIT_SUCCESS);
}
