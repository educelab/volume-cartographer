//
// Created by Seth Parker on 12/21/15.
//
#pragma once

#include "common/vc_defines.h"

namespace volcart {
    namespace meshing {
        class deepCopy {
        public:
            deepCopy(MeshType::Pointer input, MeshType::Pointer output);
        }; // deepCopy
    } // meshing
} // volcart
