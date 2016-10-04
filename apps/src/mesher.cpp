#include <iostream>
#include <string>

#include <common/types/Point.h>
#include <common/io/PointSetIO.h>
#include <common/io/plyWriter.h>
#include "meshing/OrderedPointSetMesher.h"

#include <boost/filesystem.hpp>
#include <common/types/PointSet.h>


namespace fs = boost::filesystem;

int main(int argc, char* argv[]) {
  if (argc < 2)
  {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << "orderedFile.vcps" << std::endl;
    return (1);
  }
  
  volcart::OrderedPointSet<volcart::Point3d> inputCloud;
  
 volcart::PointSetIO<volcart::Point3d >::ReadOrderedPointSet (argv[1]); //* load the file

  fs::path outfile{argv[1]};
  outfile.replace_extension("ply");
  
  volcart::meshing::OrderedPointSetMesher mesh(inputCloud);
   mesh.compute();

    VC_MeshType::Pointer output = mesh.getOutputMesh();
    volcart::io::plyWriter writer (outfile, output);

  exit(EXIT_SUCCESS);
}
