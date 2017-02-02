//
// Created by Seth Parker on 10/22/15.
//

/**@file ScaleMesh.cpp  */

#include "meshing/ScaleMesh.hpp"
#include "meshing/DeepCopy.hpp"

namespace volcart
{
namespace meshing
{

void ScaleMesh(
    const ITKMesh::Pointer& input,
    const ITKMesh::Pointer& output,
    double scaleFactor)
{
    DeepCopy(input,output);
    for (auto v = output->GetPoints()->Begin(); v != output->GetPoints()->End(); ++v) {
        v->Value()[0] *= scaleFactor;
        v->Value()[1] *= scaleFactor;
        v->Value()[2] *= scaleFactor; 
    }
};
}
}
