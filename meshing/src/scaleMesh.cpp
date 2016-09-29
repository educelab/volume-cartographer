//
// Created by Seth Parker on 10/22/15.
//

#include "meshing/scaleMesh.h"

namespace volcart {
    namespace meshing {

        void scaleMesh( MeshType::Pointer input, MeshType::Pointer output, double scale_factor ) {

            // Scale uniformly
            typedef itk::ScaleTransform<double, 3> VC_3DScaleType;
            VC_3DScaleType::Pointer scaleTransform = VC_3DScaleType::New();
            itk::FixedArray<double, 3> scale;
            scale[0] = scale_factor;
            scale[1] = scale[0]; // uniform scaling
            scale[2] = scale[0];
            scaleTransform->SetScale(scale);

            // To-Do: Translate/Set Scale origin? Unclear if this is necessary. - SP, 09-2015

            // Apply the scale
            std::cerr << "volcart::meshing::Scaling the mesh..." << std::endl;
            typedef itk::TransformMeshFilter<MeshType, MeshType, VC_3DScaleType> VC_ScaleMeshFilter;
            VC_ScaleMeshFilter::Pointer scaleFilter = VC_ScaleMeshFilter::New();
            scaleFilter->SetTransform(scaleTransform);

            scaleFilter->SetInput(input);
            scaleFilter->SetOutput(output); // This method is deprecated. Need to find a better solution. - SP, 10/2015
            scaleFilter->Update();

        };

    } // meshing
} // volcart
