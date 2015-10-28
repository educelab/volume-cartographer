//
// Created by Seth Parker on 10/22/15.
//

#include "simpleUV.h"

namespace volcart {
    namespace texturing {
        volcart::UVMap simpleUV(VC_MeshType::Pointer mesh, int width, int height) {

            volcart::UVMap uvMap;
            unsigned long pointID, meshX, meshY;
            double u, v;

            // Generate UV coord for each point in mesh
            VC_PointsInMeshIterator point = mesh->GetPoints()->Begin();
            while ( point != mesh->GetPoints()->End() ) {

                pointID = point.Index();

                // Calculate the point's [meshX, meshY] position based on its pointID
                meshX = pointID % width;
                meshY = (pointID - meshX) / width;

                // Calculate the point's UV position
                u =  (double) meshX / (double) width;
                v =  (double) meshY / (double) height;

                cv::Vec2d uv( u, v );

                // Add the uv coordinates into our map at the point index specified
                uvMap.set( pointID, uv );

                ++point;
            }

            return uvMap;
        };
    }; // texturing
}; // volcart