//
// Created by Seth Parker on 12/3/15.
//

#include "common/types/Texture.h"

using namespace volcart;

// Constructor
Texture::Texture()
{
    _metadata.set<std::string>("type", "texture");
    _metadata.set<std::string>("id", DATE_TIME());
    _metadata.set<size_t>("number-of-images", 0);
}

// Load from path
Texture::Texture(std::string path)
{
    _path = path;
    _metadata = volcart::Metadata(_path.string() + "/meta.json");

    // Check for meta-type
    if (_metadata.get<std::string>("type") != "texture") {
        std::cerr << "volcart::texture::error: metadata not of type \"texture\""
                  << std::endl;
    };

    // To-Do: Load the UV Map

    // Load the texture images
    for (size_t i_id = 0; i_id < _metadata.get<size_t>("number-of-images"); ++i_id) {
        std::string i_path =
            _path.string() + "/" + std::to_string(i_id) + ".png";
        _images.push_back(cv::imread(i_path, -1));
    }
};

// Add an image
void Texture::addImage(cv::Mat image)
{
    if (_images.empty()) {
        _width = image.cols;
        _height = image.rows;
    }
    _images.push_back(image);
    _metadata.set<size_t>("number-of-images", _images.size());
};

// Return the intensity for a Point ID
double Texture::intensity(int point_ID, int image_ID)
{
    cv::Vec2d mapping = _uvMap.get(point_ID);
    if (mapping != VC_UVMAP_NULL_MAPPING) {
        int u = cvRound(mapping[0] * (_width - 1));
        int v = cvRound(mapping[1] * (_height - 1));
        return _images[image_ID].at<unsigned short>(v, u);
    } else {
        return volcart::TEXTURE_NO_VALUE;
    }
};
