//
// Created by Seth Parker on 12/28/15.
//

#include "compositeTextureV2.h"

namespace volcart {
    namespace texturing {

        // Constructor
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

        // Do the hard work
        int compositeTextureV2::_process() {

            // Auto-generate minor radius for elliptical search
            double searchMinorRadius;
            if( (searchMinorRadius = _radius / 3) < 1 ) searchMinorRadius = 1;

            // Generate homographies
            _generateCellInfo();

            // Iterate over every pixel in the output image
            unsigned long pixelsNotInCell = 0;
            cv::Mat image( _height, _width, CV_16UC1 );
            VC_PointsLocatorType::NeighborsIdentifierType neighborhood;
            for ( int y = 0; y < _height; ++y ) {
                for ( int x = 0; x < _width; ++x ) {
                    double progress = (double)(x + 1 + (_width*y))/(double)(_width*_height) * (double) 100;
                    std::cerr << "volcart::texturing::compositeTexturing: Generating texture: " << std::to_string( progress ) << "%\r" << std::flush;

                    // This pixel's uv coordinate
                    cv::Vec3d uv( 0, 0, 0 );
                    uv[0] = (double) x / (double) ( _width - 1 );
                    uv[1] = (double) y / (double) ( _height - 1);

                    // Empty our averaging variables
                    if ( !neighborhood.empty() ) neighborhood.clear();

                    // find k nearest neighbors for current point
                    VC_PointType searchPoint;
                    searchPoint[0] = uv[0]; searchPoint[1] = uv[1]; searchPoint[2] = 0.0;
                    _kdTree->FindClosestNPoints( searchPoint, 100, neighborhood );

                    // Find which triangle this pixel lies inside of
                    bool in2D = false; // Is the current pixel in this cell?
                    cellInfo info;
                    cv::Vec3d baryCoord(0, 0, 0);
                    for (auto c_id = neighborhood.begin(); c_id != neighborhood.end(); ++c_id) {
                        info = _cellInformation[*c_id];

                        // Calculate the 3D position of this pixel using the homography matrix
                        baryCoord = _BarycentricCoord(uv, info.Pts2D[0], info.Pts2D[1], info.Pts2D[2] );
                        in2D = (baryCoord[0] >= 0 && baryCoord[1] >= 0 && baryCoord[2] >= 0 && baryCoord[0] + baryCoord[1] <= 1 );

                        if ( in2D ) break;
                    }

                    // Set this pixel to black if not part of a cell
                    if ( !in2D ) {
                        image.at< unsigned short >(y, x) = 0;
                        ++pixelsNotInCell;
                        continue;
                    }

                    // Find the xyz coordinate of the original point
                    cv::Vec3d xyz = _CartesianCoord(baryCoord, info.Pts3D[0], info.Pts3D[1], info.Pts3D[2] );

                    // Use the cell normal as the normal for this point
                    cv::Vec3d xyz_norm = info.Normal;

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
            std::cerr << "volcart::texturing::compositeTexture:: Pixels not in cell: " << pixelsNotInCell << std::endl;

            // Set output
            _texture.addImage(image);
            _texture.uvMap( _uvMap );

            return EXIT_SUCCESS;
        };

        // Calculate homography matrices
        int compositeTextureV2::_generateCellInfo() {

            // Make sure the storage vectors are clean
            if ( !_cellInformation.empty() ) _cellInformation.clear();
            _cellCentroids = VC_MeshType::New();
            _kdTree = VC_PointsLocatorType::New();

            // Generate a homography matrix for each cell in the mesh
            VC_PointType centroid;
            std::cerr << "volcart::texturing::compositeTexturing: Generating cell information" << std::endl;
            for ( auto cell = _input->GetCells()->Begin(); cell != _input->GetCells()->End(); ++cell ) {
                cellInfo info = cellInfo();
                cv::Vec3d _2D, _3D;
                for( VC_PointsInCellIterator point = cell->Value()->PointIdsBegin(); point != cell->Value()->PointIdsEnd(); ++point ) {
                    unsigned long pointID = *point;

                    _2D[0] = _uvMap.get(pointID)[0];
                    _2D[1] = _uvMap.get(pointID)[1];
                    _2D[2] = 0.0;
                    info.Pts2D.push_back(_2D);

                    _3D[0] = _input->GetPoint(pointID)[0];
                    _3D[1] = _input->GetPoint(pointID)[1];
                    _3D[2] = _input->GetPoint(pointID)[2];
                    info.Pts3D.push_back(_3D);
                }

                // Calculate the cell centroid
                cv::Vec3d temp_cent = (info.Pts2D[0] + info.Pts2D[1] + info.Pts2D[2]) / 3;
                centroid[0] = temp_cent[0]; centroid[1] = temp_cent[1]; centroid[2] = temp_cent[2];

                // Generate the surface normal for this cell
                cv::Vec3d v1v0 = info.Pts3D[1] - info.Pts3D[0];
                cv::Vec3d v2v0 = info.Pts3D[2] - info.Pts3D[0];
                info.Normal = cv::normalize( v1v0.cross(v2v0) );

                _cellInformation.push_back( info );
                _cellCentroids->SetPoint(cell.Index(), centroid );
            }

            _kdTree->SetPoints(_cellCentroids->GetPoints());
            _kdTree->Initialize();

            return EXIT_SUCCESS;
        }

        // Find barycentric coordinates of point in triangle
        // From Christer Ericson's Real-Time Collision Detection
        // Code from: http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
        cv::Vec3d compositeTextureV2::_BarycentricCoord( const cv::Vec3d &nXYZ,
                                                         const cv::Vec3d &nA,
                                                         const cv::Vec3d &nB,
                                                         const cv::Vec3d &nC )
        {
            cv::Vec3d v0 = nB - nA;
            cv::Vec3d v1 = nC - nA;
            cv::Vec3d v2 = nXYZ - nA;

            double dot00 = v0.dot(v0);
            double dot01 = v0.dot(v1);
            double dot11 = v1.dot(v1);
            double dot20 = v2.dot(v0);
            double dot21 = v2.dot(v1);
            double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);

            cv::Vec3d output;
            output[1] = (dot11 * dot20 - dot01 * dot21) * invDenom;
            output[2] = (dot00 * dot21 - dot01 * dot20) * invDenom;
            output[0] = 1.0 - output[1] - output[2];

            return output;
        }

        // Find Cartesian coordinates of point in triangle given barycentric coordinate
        cv::Vec3d compositeTextureV2::_CartesianCoord( const cv::Vec3d &nUVW,
                                                       const cv::Vec3d &nA,
                                                       const cv::Vec3d &nB,
                                                       const cv::Vec3d &nC )
        {
            return nUVW[0] * nA + nUVW[1] * nB + nUVW[2] * nC;
        }

    }
}