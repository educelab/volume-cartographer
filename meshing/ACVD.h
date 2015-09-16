//
// Created by Media Team on 9/16/15.
//

#include <vtkPolyDataNormals.h>
#include "vtkIsotropicDiscreteRemeshing.h"

#include "vc_defines.h"
#include "itk2vtk.h"

#ifndef ACVD_H
#define ACVD_H

namespace volcart {
	namespace meshing {
		VC_MeshType::Pointer ACVD ( VC_MeshType::Pointer  inputMesh,
                                		int                   NumberOfSamples,
                                		float                 Gradation );
	}
}

#endif // ACVD_H