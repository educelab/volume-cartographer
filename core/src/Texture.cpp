//
// Created by Seth Parker on 12/3/15.
//
#include <opencv2/imgcodecs.hpp>

#include "core/types/Texture.hpp"

namespace fs = boost::filesystem;
using namespace volcart;

// Constructor
Texture::Texture()
{
    metadata_.set<std::string>("type", "texture");
    metadata_.set<std::string>("id", DateTime());
    metadata_.set<size_t>("number-of-images", 0);
}

// Load from path
Texture::Texture(fs::path path)
    : metadata_{path / "meta.json"}, path_{std::move(path)}
{
    // Check for meta-type
    if (metadata_.get<std::string>("type") != "texture") {
        std::cerr << "volcart::texture::error: metadata not of type 'texture'"
                  << std::endl;
    }

    // To-Do: #180

    // Load the texture images
    for (size_t i = 0; i < metadata_.get<size_t>("number-of-images"); ++i) {
        auto imagePath = path_ / (std::to_string(i) + ".png");
        images_.push_back(cv::imread(imagePath.string(), cv::IMREAD_ANYDEPTH));
    }
}

// Add an image
void Texture::addImage(cv::Mat image)
{
    if (images_.empty()) {
        width_ = image.cols;
        height_ = image.rows;
    }
    images_.push_back(image);
    metadata_.set<size_t>("number-of-images", images_.size());
}

// Return the intensity for a Point ID
double Texture::intensity(int pointId, int imageId)
{
    auto mapping = ppm_.uvMap().get(pointId);
    if (mapping != VC_UVMAP_NULL_MAPPING) {
        int u = cvRound(mapping[0] * (width_ - 1));
        int v = cvRound(mapping[1] * (height_ - 1));
        return images_[imageId].at<uint16_t>(v, u);
    } else {
        return volcart::TEXTURE_NO_VALUE;
    }
}
