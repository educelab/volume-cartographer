//
// Created by Seth Parker on 10/20/15.
//

#ifndef VC_TEXTURE_H
#define VC_TEXTURE_H

#include <opencv2/opencv.hpp>

#include "../vc_defines.h"
#include "UVMap.h"

namespace volcart {
    class Texture {
    public:
        Texture(){};
        Texture(int width, int height) { _width = width; _height = height; };

        // Get dimensions
        int width()  { return _width; };
        int height() { return _height; };

        // Get/Set UV Map
        volcart::UVMap& uvMap(){ return _uvMap; };
        void uvMap(volcart::UVMap uvMap) { _uvMap = uvMap; };

        // Get/Add Texture Image
        cv::Mat getImage(int id) { return _images[id]; };
        void addImage(cv::Mat image) { _images.push_back(image); };

    private:
        int _width, _height;
        std::vector<cv::Mat> _images;
        volcart::UVMap _uvMap;
    };
} // volcart

#endif //VC_TEXTURE_H
