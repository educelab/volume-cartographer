// meshUtils.h
// Abigail Coleman June 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <vector>

#include "vc_defines.h"

#ifndef VC_MESHUTILS_H
#define VC_MESHUTILS_H

VC_MeshType::Pointer smoothNormals ( VC_MeshType::Pointer  inputMesh,
                                     double                smoothingFactor );

#endif // VC_MESHUTILS_H
