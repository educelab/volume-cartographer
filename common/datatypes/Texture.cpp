//
// Created by Seth Parker on 12/3/15.
//

#include "Texture.h"

namespace volcart {

    // Constructor
    Texture::Texture() {
        _metadata.setValue( "type", "texture" );
        _metadata.setValue( "id", VC_DATE_TIME() );
        _metadata.setValue( "number-of-images", 0);
    };

    // Load from path
    Texture::Texture(std::string path) {
        _path = path;
        _metadata = volcart::Metadata( _path.string() + "/meta.json" );

        // Check for meta-type
        if ( _metadata.getString( "type" ) != "texture" ) {
            std::cerr << "volcart::texture::error: metadata not of type \"texture\"" << std::endl;
        };

        // To-Do: Load the UV Map

        // Load the texture images
        for ( int i_id = 0; i_id < _metadata.getInt("number-of-images"); ++i_id ) {
            std::string i_path = _path.string() + "/" + std::to_string(i_id) + ".png";
            _images.push_back( cv::imread( i_path, -1 ) );
        }
    };

    // Add an image
    void Texture::addImage(cv::Mat image) {
        if ( _images.empty() ) {
            _width = image.cols;
            _height = image.rows;
        }
        _images.push_back(image);
        _metadata.setValue( "number-of-images", _images.size() );
    };

    // Return the intensity for a Point ID
    double Texture::intensity( int point_ID, int image_ID ) {
        cv::Vec2d mapping = _uvMap.get(point_ID);
        if ( mapping != VC_UVMAP_NULL_MAPPING ) {
            int u =  cvRound(mapping[0] * (_width - 1));
            int v =  cvRound(mapping[1] * (_height - 1));
            return _images[image_ID].at< unsigned short > ( v, u );
        } else {
            return VC_TEXTURE_NO_VALUE;
        }
    };
}