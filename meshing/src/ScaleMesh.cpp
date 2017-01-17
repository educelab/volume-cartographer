//
// Created by Seth Parker on 10/22/15.
//

/**@file ScaleMesh.cpp  */

#include "meshing/ScaleMesh.hpp"

namespace volcart
{
namespace meshing
{

void ScaleMesh(
    const ITKMesh::Pointer& input,
    const ITKMesh::Pointer& output,
    double scaleFactor)
{
    // Scale uniformly
    using VC3DScaleType = itk::ScaleTransform<double, 3>;
    auto scaleTransform = VC3DScaleType::New();
    itk::FixedArray<double, 3> scale;
    scale[0] = scaleFactor;
    scale[1] = scale[0];  // uniform scaling
    scale[2] = scale[0];
    scaleTransform->SetScale(scale);

    // To-Do: Translate/Set Scale origin? Unclear if this is necessary. - SP,
    // 09-2015

    // Apply the scale
    std::cerr << "volcart::meshing::Scaling the mesh..." << std::endl;
    using VCScaleMeshFilter =
        itk::TransformMeshFilter<ITKMesh, ITKMesh, VC3DScaleType>;
    auto scaleFilter = VCScaleMeshFilter::New();
    scaleFilter->SetTransform(scaleTransform);
    scaleFilter->SetInput(input);

    // This method is deprecated. Need to find a better solution. - SP, 10/2015
    scaleFilter->SetOutput(output);
    scaleFilter->Update();
};
}
}
