// smoothNormals.h
// Abigail Coleman June 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <vector>

#include "vc_defines.h"

#ifndef VC_SMOOTHNORMALS_H
#define VC_SMOOTHNORMALS_H

namespace volcart {
    namespace meshing {
        VC_MeshType::Pointer smoothNormals ( VC_MeshType::Pointer  inputMesh,
                                             double                smoothingFactor );
    }
}

#endif // VC_SMOOTHNORMALS_H
