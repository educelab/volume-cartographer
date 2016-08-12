// smoothNormals.h
// Abigail Coleman June 2015

#ifndef VC_SMOOTHNORMALS_H
#define VC_SMOOTHNORMALS_H

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <vector>

#include "common/vc_defines.h"
#include "meshing/deepCopy.h"

namespace volcart {
    namespace meshing {
        VC_MeshType::Pointer smoothNormals ( VC_MeshType::Pointer input,
                                             double               radius);
    }
}

#endif // VC_SMOOTHNORMALS_H
