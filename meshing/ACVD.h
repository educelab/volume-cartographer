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
		// input mesh is an itk mesh passed to be remeshed using ACVD
		// NumberOfSamples is the desired number of vertices
		// gradation is the gradation parameter (default value: 0 is uniform, higher values give more 
		// 																			 and more importance to regions with high curvature)
		// ConsoleOutput sets the graphics display (0 : no display. 1: display. 2 :iterative display)
		//																					default value : 1
		// SubsamplingThreshold: Higher values give better results but the input mesh will be 
		//											 subdivided more times
		VC_MeshType::Pointer ACVD ( VC_MeshType::Pointer  inputMesh,
                                int  		              NumberOfSamples,
                                float    		          Gradation = 0,
                                int 									ConsoleOutput = 0,
                                int 									SubsamplingThreshold = 10);
	}
}

#endif // ACVD_H