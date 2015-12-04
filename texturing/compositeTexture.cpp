//
// Created by Seth Parker on 10/20/15.
//

#include "compositeTexture.h"

namespace volcart {
    namespace texturing {
        volcart::Texture compositeTexture( VC_MeshType::Pointer inputMesh,
                                           VolumePkg volpkg,
                                           int output_w,
                                           int output_h,
                                           double searchMajorRadius,
                                           VC_Composite_Option compositeMethod,
                                           VC_Direction_Option compositeDirection) {

            ///// Create the output texture object /////
            volcart::Texture outputTexture;

            ///// Generate UV Map /////
            // To-Do: Generate this map independent of point ordering - SP, 10/2015

            volcart::UVMap uvMap;
            int meshWidth = output_w; int meshHeight = output_h;

            uvMap = volcart::texturing::simpleUV(inputMesh, meshWidth, meshHeight);

            ///// Generate Texture Image /////

            cv::Mat textureImage = cv::Mat::zeros( output_h, output_w, CV_16UC1 );

            // Auto-generate minor radius for elliptical search
            double searchMinorRadius;
            if( (searchMinorRadius = searchMajorRadius / 3) < 1 ) searchMinorRadius = 1;

            // Initialize iterators
            VC_CellIterator  cellIterator = inputMesh->GetCells()->Begin();
            VC_CellIterator  cellEnd      = inputMesh->GetCells()->End();
            VC_CellType *    cell;
            VC_PointsInCellIterator pointsIterator;

            unsigned long pointID;
            double u, v;

            // Iterate over all of the cells to lay out the faces in the output texture
            for( ; cellIterator != cellEnd; ++cellIterator ) {
                // Link the pointer to our current cell
                cell = cellIterator.Value();

                std::cout << "Texturing face " << cellIterator.Index() << "/" << cellEnd.Index() << "\r" << std::flush;

                // Iterate over the vertices of the current cell
                pointsIterator = cell->PointIdsBegin();
                for( ; pointsIterator != cell->PointIdsEnd(); ++pointsIterator ) {
                    pointID = *pointsIterator;

                    VC_PointType p = inputMesh->GetPoint(pointID);
                    VC_PixelType normal;
                    inputMesh->GetPointData( pointID, &normal );

                    // Fill in the output pixel with a value
                    // cv::Mat.at uses (row, column)
                    double value = textureWithMethod( cv::Vec3f(p[0], p[1], p[2]),
                                                      cv::Vec3f(normal[0], normal[1], normal[2]),
                                                      volpkg,
                                                      compositeMethod,
                                                      searchMajorRadius,
                                                      searchMinorRadius,
                                                      0.5,
                                                      compositeDirection);

                    // Retrieve the point's uv position from the UV Map
                    u =  cvRound(uvMap.get(pointID)[0] * output_w);
                    v =  cvRound(uvMap.get(pointID)[1] * output_h);

                    // Assign the intensity value at the UV position
                    textureImage.at < unsigned short > (v, u) = (unsigned short) value;
                }
            }
            std::cout << std::endl;

            // Assign and return the output
            outputTexture.addImage(textureImage);
            outputTexture.uvMap(uvMap);
            return outputTexture;

        };
    } // texturing
} // volcart