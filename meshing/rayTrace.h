//
// Created by Media Team on 8/12/15.
//

#include <cstdio>

#include <opencv2/opencv.hpp>

#include <vtkSmartPointer.h>
#include <vtkOBBTree.h>
#include <vtkPolyDataNormals.h>

#include "itk2vtk.h"

#ifndef VC_RAYTRACE_H
#define VC_RAYTRACE_H

namespace volcart {
  namespace meshing {
  	// Using vtk's OBBTree to test a ray's intersection with the faces/cells/triangles in the mesh
  	std::vector< std::vector<cv::Vec3f> > rayTrace(VC_MeshType::Pointer itkMesh, int aTraceDir, int &width, int &height);
  }
}

#endif //VC_RAYTRACE_H