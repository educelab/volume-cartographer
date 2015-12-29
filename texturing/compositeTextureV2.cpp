//
// Created by Seth Parker on 12/28/15.
//

#include "compositeTextureV2.h"

namespace volcart {
    namespace texturing {

        compositeTextureV2::compositeTextureV2(VC_MeshType::Pointer inputMesh,
                                               VolumePkg &volpkg,
                                               UVMap uvMap,
                                               double radius,
                                               int width,
                                               int height,
                                               VC_Composite_Option method,
                                               VC_Direction_Option direction) :
                _volpkg(volpkg), _input(inputMesh), _uvMap(uvMap), _radius(radius), _width(width), _height(height), _method(method), _direction(direction)
        {

            _process();
        };

        int compositeTextureV2::_process() {

            // Auto-generate minor radius for elliptical search
            double searchMinorRadius;
            if( (searchMinorRadius = _radius / 3) < 1 ) searchMinorRadius = 1;

            // Generate homographies
            _generateHomographies();

            // Output
            unsigned long pixelsNotInCell = 0;
            cv::Mat image( _height, _width, CV_16UC1 );
            for ( int y = 0; y < _height; ++y ) {
                for ( int x = 0; x < _width; ++x ) {
                    double progress = (double)(x + 1 + (_width*y))/(double)(_width*_height) * (double) 100;
                    std::cerr << "volcart::texturing::compositeTexturing: Generating texture: " << std::to_string( progress ) << "%\r" << std::flush;

                    // This pixel's uv coordinate
                    cv::Vec3d uv( 0, 0, 1 );
                    uv[0] = (double) x / (double) _width;
                    uv[1] = (double) y / (double) _height;

                    // Find which triangle this pixel belongs to
                    bool inCell = false;
                    unsigned long cell_id = 0;

                    std::vector<cv::Vec3d> vertices2D;
                    std::vector<cv::Vec3d> vertices3D;
                    cv::Vec3d pos_2D, pos_3D;
                    for ( auto cell = _input->GetCells()->Begin(); cell != _input->GetCells()->End(); ++cell ) {
                        for (VC_PointsInCellIterator point = cell->Value()->PointIdsBegin(); point != cell->Value()->PointIdsEnd(); ++point) {
                            pos_3D[0] = _input->GetPoint(*point)[0];
                            pos_3D[1] = _input->GetPoint(*point)[1];
                            pos_3D[2] = _input->GetPoint(*point)[2];
                            vertices3D.push_back(pos_3D);

                            pos_2D[0] = _uvMap.get(*point)[0];
                            pos_2D[1] = _uvMap.get(*point)[1];
                            pos_2D[2] = 1.0;
                            vertices2D.push_back(pos_2D);
                        }

                        inCell = checkPtInTriangleUtil::IsPtInTriangle( uv, vertices2D[0], vertices2D[1], vertices2D[2] );
                        if ( inCell ) {
                            cell_id = cell->Index();
                            break;
                        }

                        // Empty the vector for the next cell
                        vertices3D.clear();
                        vertices2D.clear();
                    }

                    // Set this pixel to black if not part of a cell
                    if ( !inCell ) {
                        image.at < unsigned short > (y, x) = 0;
                        ++pixelsNotInCell;
                        continue;
                    }

                    // Lookup the original 3D position
                    cv::Vec3d xyz = CalcMappedPoint( uv, _homographies[cell_id] );

                    // Calculate the cell's normal for this normal
                    cv::Vec3d v1v0 = vertices3D[1] - vertices3D[0];
                    cv::Vec3d v2v0 = vertices3D[2] - vertices3D[0];
                    cv::Vec3d xyz_norm = cv::normalize( v1v0.cross(v2v0) );

                    // Generate the intensity value
                    double value = textureWithMethod( xyz,
                                                      xyz_norm,
                                                      _volpkg,
                                                      _method,
                                                      _radius,
                                                      searchMinorRadius,
                                                      0.5,
                                                      _direction);

                    // Assign the intensity value at the UV position
                    image.at< unsigned short >(y, x) = (unsigned short) value;
                }
            }
            std::cerr << std::endl;
            std::cerr << "volcart::compositeTexture::pixels not in cell: " << pixelsNotInCell << std::endl;

            // Set output
            _texture.addImage(image);
            _texture.uvMap( _uvMap );

            return EXIT_SUCCESS;
        };

        // Calculate homography matrices
        int compositeTextureV2::_generateHomographies() {

            // Make sure the storage vector is clean
            if ( !_homographies.empty() ) _homographies.clear();

            // Generate a homography matrix for each cell in the mesh
            std::vector<cv::Vec3d> _2DPts, _3DPts;
            cv::Vec3d _2D, _3D;
            std::cerr << "volcart::texturing::compositeTexturing: Generating homographies" << std::endl;
            for ( auto cell = _input->GetCells()->Begin(); cell != _input->GetCells()->End(); ++cell ) {

                for( VC_PointsInCellIterator point = cell->Value()->PointIdsBegin(); point != cell->Value()->PointIdsEnd(); ++point ) {
                    unsigned long pointID = *point;

                    _2D[0] = _uvMap.get(pointID)[0];
                    _2D[1] = _uvMap.get(pointID)[1];
                    _2D[2] = 1.0;
                    _2DPts.push_back(_2D);

                    _3D[0] = _input->GetPoint(pointID)[0];
                    _3D[1] = _input->GetPoint(pointID)[1];
                    _3D[2] = _input->GetPoint(pointID)[2];
                    _3DPts.push_back(_3D);
                }

                cv::Mat _homography(3, 3, CV_64F);
                CalcHomographyFromPoints( _2DPts, _3DPts, _homography);

                _homographies.push_back( _homography );

                _2DPts.clear();
                _3DPts.clear();
            }

            return EXIT_SUCCESS;
        }

        //

    }
}