///// Per-Pixel Map /////
//
// Effectively a raster of a UV Map for reverse lookups.
// Every pixel in the map holds the 3D position and normal
// used to generate that pixel's intensity in the corresponding
// texture image. Useful for regenerating textures with differing
// parameters or for identifying where in a volume a particular
// came from.
//
// Created by Seth Parker on 3/17/16.


#ifndef VC_PERPIXELMAP_H
#define VC_PERPIXELMAP_H

namespace volcart {
    class PerPixelMap {
    public:
        ///// Constructors /////
        // Empty Map of width x height
        PerPixelMap( int width, int height ) : _width(width), _height(height) {
          _map = cv::Mat_<cv::Vec6d>( height, width, cv::Vec6d(0,0,0,0,0,0) );
        }

        // Construct map from file
        PerPixelMap( std::string path ) {
          read(path);
        }

        ///// Operators /////
        // Forward to the Mat_ operators
        cv::Vec6d& operator ()( int x, int y ) { return _map(y,x); };

        ///// Metadata /////
        int width() { return _width; };
        int height() { return _height; };

        ///// Disk IO /////
        void write( std::string path );
        void read( std::string path );

    private:
        int _width, _height;
        cv::Mat_<cv::Vec6d> _map;
    };
}


#endif //VC_PERPIXELMAP_H
