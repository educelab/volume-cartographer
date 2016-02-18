//
// Created by Seth Parker on 10/20/15.
//

#include "compositeTexture.h"

namespace volcart {
    namespace texturing {
        compositeTexture::compositeTexture( VC_MeshType::Pointer inputMesh,
                                            VolumePkg& volpkg,
                                            int output_w,
                                            int output_h,
                                            double radius,
                                            VC_Composite_Option compositeMethod,
                                            VC_Direction_Option compositeDirection) :
        _volpkg(volpkg), _input(inputMesh), _width(output_w), _height(output_h), _radius(radius),
        _method(compositeMethod), _direction(compositeDirection)
        {
            ///// Generate UV Map /////
            // To-Do: Generate this map independent of point ordering - SP, 10/2015
            _uvMap = volcart::texturing::simpleUV(inputMesh, _width, _height);

            _process();
        };

        compositeTexture::compositeTexture(VC_MeshType::Pointer inputMesh,
                                           VolumePkg &volpkg,
                                           UVMap uvMap,
                                           double radius,
                                           VC_Composite_Option method,
                                           VC_Direction_Option direction) :
        _volpkg(volpkg), _input(inputMesh), _uvMap(uvMap), _radius(radius), _method(method), _direction(direction)
        {
            _width = uvMap.ratio().width;
            _height = uvMap.ratio().height;

            _process();
        }

        int compositeTexture::_process() {

            ///// Generate Texture Image /////
            cv::Mat textureImage = cv::Mat::zeros( _height, _width, CV_16UC1 );

            // Auto-generate minor radius for elliptical search
            double searchMinorRadius;
            if( (searchMinorRadius = _radius / 3) < 1 ) searchMinorRadius = 1;

            // Initialize iterators
            VC_CellIterator  cellIterator = _input->GetCells()->Begin();
            VC_CellIterator  cellEnd      = _input->GetCells()->End();
            VC_CellType *    cell;
            VC_PointsInCellIterator pointsIterator;

            unsigned long pointID;
            double u, v;

            // Iterate over all of the cells to lay out the faces in the output texture
            for( ; cellIterator != cellEnd; ++cellIterator ) {
                // Link the pointer to our current cell
                cell = cellIterator.Value();

                std::cerr << "volcart::compositeTexture::message: Texturing face " << cellIterator.Index() << "/" << cellEnd.Index() << "\r" << std::flush;

                // Iterate over the vertices of the current cell
                pointsIterator = cell->PointIdsBegin();
                for( ; pointsIterator != cell->PointIdsEnd(); ++pointsIterator ) {
                    pointID = *pointsIterator;

                    VC_PointType p = _input->GetPoint(pointID);
                    VC_PixelType normal;
                    _input->GetPointData( pointID, &normal );

                    // Fill in the output pixel with a value
                    // cv::Mat.at uses (row, column)
                    double value = textureWithMethod( cv::Vec3f(p[0], p[1], p[2]),
                                                      cv::Vec3f(normal[0], normal[1], normal[2]),
                                                      _volpkg,
                                                      _method,
                                                      _radius,
                                                      searchMinorRadius,
                                                      0.5,
                                                      _direction);

                    // Retrieve the point's uv position from the UV Map
                    u =  cvRound(_uvMap.get(pointID)[0] * _width);
                    v =  cvRound(_uvMap.get(pointID)[1] * _height);

                    // Assign the intensity value at the UV position
                    textureImage.at < unsigned short > (v, u) = (unsigned short) value;
                }
            }
            std::cout << std::endl;

            // Assign and return the output
            _texture.addImage(textureImage);
            _texture.uvMap(_uvMap);

            return EXIT_SUCCESS;
        }
    } // texturing
} // volcart