//
// Created by Media Team on 9/16/15.
//

#include "vc_defines.h"

#ifndef SMOOTHNORMALS_H
#define SMOOTHNORMALS_H

namespace volcart {
	namespace meshing {

		VC_MeshType::Pointer smoothNormals ( VC_MeshType::Pointer  inputMesh,
                                     		 double                smoothingFactor );
	}
}

#endif // SMOOTHNORMALS_H