//
// Created by Abigail Coleman 2/3/16
//

#ifndef VC_CLOTH_H
#define VC_CLOTH_H

#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>

#include <vtkSmoothPolyDataFilter.h>
#include <vtkPLYReader.h>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"
#include "io/objWriter.h"
#include "compositeTextureV2.h"
#include "ACVD.h"
#include "deepCopy.h"

// bullet converter
#include "itk2bullet.h"
#include <LinearMath/btVector3.h>
#include <vtkQuadricDecimation.h>

namespace volcart {
	namespace meshing {
		class Cloth {
		public:

		private:
		};
	}

}

#endif // VC_CLOTH_H