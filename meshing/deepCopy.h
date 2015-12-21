//
// Created by Seth Parker on 12/21/15.
//

#ifndef VC_DEEPCOPY_H
#define VC_DEEPCOPY_H

#include "vc_defines.h"

namespace volcart {
    namespace meshing {
        class deepCopy {
        public:
            deepCopy(VC_MeshType::Pointer input, VC_MeshType::Pointer output);
        }; // deepCopy
    } // meshing
} // volcart

#endif //VC_DEEPCOPY_H
