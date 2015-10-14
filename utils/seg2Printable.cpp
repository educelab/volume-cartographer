//
// Created by Seth Parker on 9/15/15.
//

#include <stdio.h>

#include "vc_defines.h"
#include "volumepkg.h"
#include "io/ply2itk.h"
#include "io/objWriter.h"

#include <itkScaleTransform.h>
#include <itkTransformMeshFilter.h>

int main (int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " volpkg seg-id" << std::endl;
        return EXIT_FAILURE;
    }

    // Load our sources from the volpkg
    VolumePkg volpkg = VolumePkg(argv[1]);
    volpkg.setActiveSegmentation(argv[2]);

    std::string outputName = argv[2];
    outputName += "_mm.obj";

    // Get our mesh
    VC_MeshType::Pointer mesh = VC_MeshType::New();
    int width = -1, height = -1;
    if (!volcart::io::ply2itkmesh(volpkg.getMeshPath(), mesh, width, height)) {
        std::cerr << "ERROR: Could not read ply file" << std::endl;
        return EXIT_FAILURE;
    }

    // Scale uniformly
    typedef itk::ScaleTransform<double, 3> VC_3DScaleType;
    VC_3DScaleType::Pointer scaleTransform = VC_3DScaleType::New();
    itk::FixedArray<double, 3> scale;
    scale[0] = volpkg.getVoxelSize() * 0.001; // coord * voxelsize um * 0.001 mm/um = coord mm
    scale[1] = scale[0]; // uniform scaling
    scale[2] = scale[0];
    scaleTransform->SetScale(scale);

    // To-Do: Translate/Set Scale origin? Unclear if this is necessary. - SP, 09-2015

    // Apply the scale
    std::cout << "Scaling the mesh..." << std::endl;
    typedef itk::TransformMeshFilter<VC_MeshType, VC_MeshType, VC_3DScaleType> VC_ScaleMeshFilter;
    VC_ScaleMeshFilter::Pointer scaleFilter = VC_ScaleMeshFilter::New();
    scaleFilter->SetInput(mesh);
    scaleFilter->SetTransform(scaleTransform);
    scaleFilter->Update();

    // Write the mesh
    std::cout << "Writing scaled mesh..." << std::endl;
    volcart::io::objWriter objWriter(outputName, scaleFilter->GetOutput());
    objWriter.write();
}