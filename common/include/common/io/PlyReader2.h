//
// Created by Hannah Hatch on 10/18/16.
//

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <string.h>

#include "boost/filesystem/path.hpp"
#include "common/vc_defines.h"
namespace volcart {
    namespace io{
        bool PLYReader2(boost::filesystem::path path, volcart::ITKMesh::Pointer mesh);
    }
}
