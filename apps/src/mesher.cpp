#include <iostream>
#include <string>

#include <common/types/Point.h>
#include <common/io/PointSetIO.h>
#include <common/io/plyWriter.h>
#include "meshing/OrderedPointSetMesher.h"

#include <boost/filesystem.hpp>
#include <common/types/PointSet.h>


namespace fs = boost::filesystem;
namespace vc = volcart;

int main(int argc, char* argv[]) {
  if (argc < 2)
  {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << "orderedFile.vcps" << std::endl;
    return (1);
  }

  // Load the file
  auto inputCloud = vc::PointSetIO<vc::Point3d >::ReadOrderedPointSet(argv[1]);

  // Set the output
  fs::path outfile{argv[1]};
  outfile.replace_extension("ply");

  vc::meshing::OrderedPointSetMesher mesh(inputCloud);
  mesh.compute();

  auto output = mesh.getOutputMesh();
  vc::io::plyWriter writer (outfile, output);

  exit(EXIT_SUCCESS);
}
