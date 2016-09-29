// smoothNormals.h
// Abigail Coleman June 2015
#pragma once

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <vector>

#include "common/vc_defines.h"
#include "meshing/deepCopy.h"

namespace volcart {
    namespace meshing {
        ITKMesh::Pointer smoothNormals ( ITKMesh::Pointer input,
                                             double               radius);
    }
}
