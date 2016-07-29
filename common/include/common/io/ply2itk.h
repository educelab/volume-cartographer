//
// Created by Seth Parker on 4/27/15.
//
#ifndef VC_PLYHELPER_H
#define VC_PLYHELPER_H

#include <iostream>
#include <fstream>
#include <cmath>

#include "../vc_defines.h"

namespace volcart {
namespace io {
    bool ply2itkmesh (std::string plyPath,
                      VC_MeshType::Pointer mesh,
                      int &width,
                      int &height);
}
}

#endif //VC_PLYHELPER_H
