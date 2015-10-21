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
            volcart::Texture outputTexture(output_w, output_h);

            //////////////////// BEGINNING OF HACK ////////////////////

            //// Load the slices from the volumepkg
            //// This block is a hack until VolumePkg can handle caching slice data.
            //// See Issue #12 for more details.

                std::vector< cv::Mat > aImgVol;

                // Setup
                int meshLowIndex = (int) inputMesh->GetPoint(0)[2];
                int meshHighIndex = meshLowIndex + output_h;
                int aNumSlices = volpkg.getNumberOfSlices();

                int bufferLowIndex = meshLowIndex - (int) searchMajorRadius;
                if (bufferLowIndex < 0) bufferLowIndex = 0;

                int bufferHighIndex = meshHighIndex + (int) searchMajorRadius;
                if (bufferHighIndex >= volpkg.getNumberOfSlices()) bufferHighIndex = volpkg.getNumberOfSlices();

                // Slices must be loaded into aImgVol at the correct index: slice 005 == aImgVol[5]
                // To avoid loading the whole volume, pad the beginning indices with 1x1 null mats
                cv::Mat nullMat = cv::Mat::zeros(1, 1, CV_16U);
                for ( int i = 0; i < bufferLowIndex; ++i ) {
                    std::cout << "\rLoading null buffer slices: " << i + 1 << "/" << bufferLowIndex << std::flush;
                    aImgVol.push_back( nullMat.clone() );
                }
                std::cout << std::endl;

                // Load the actual volume into a tempVol with a buffer of nRadius
                for ( int i = bufferLowIndex; i < bufferHighIndex; ++i ) {
                    std::cout << "\rLoading real slices: " << i - bufferLowIndex + 1 << "/" << bufferHighIndex - bufferLowIndex << std::flush;
                    aImgVol.push_back( volpkg.getSliceData( i ).clone() );
                }
                std::cout << std::endl;

            /////////////////////// END OF HACK ///////////////////////


            ///// Generate UV Map /////
            // To-Do: Generate this map independent of point ordering - SP, 10/2015

            volcart::UVMap uvMap;
            unsigned long pointID, meshX, meshY;
            int meshWidth = output_w; int meshHeight = output_h;
            double u, v;

            // Generate UV coord for each point in mesh
            VC_PointsInMeshIterator point = inputMesh->GetPoints()->Begin();
            while ( point != inputMesh->GetPoints()->End() ) {

                pointID = point.Index();

                // Calculate the point's [meshX, meshY] position based on its pointID
                meshX = pointID % meshWidth;
                meshY = (pointID - meshX) / meshWidth;

                // Calculate the point's UV position
                u =  (double) meshX / (double) meshWidth;
                v =  (double) meshY / (double) meshHeight;

                cv::Vec2d uv( u, v );

                // Add the uv coordinates into our map at the point index specified
                uvMap.set( pointID, uv );

                ++point;
            }

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
            pointID = 0;

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
                                                      aImgVol,
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