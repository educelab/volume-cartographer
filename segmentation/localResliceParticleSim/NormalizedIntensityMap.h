#pragma once

#include <utility>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace volcart {

namespace segmentation {

// A class representing the intensity map generated from a row of a matrix
// normalized to the range [0, 1]
class NormalizedIntensityMap {
public:
    NormalizedIntensityMap(cv::Mat);

    void draw(const uint32_t, const uint32_t) const;

    std::vector<std::pair<uint32_t, double>> findNMaxima(const uint32_t n=4) const;

private:
    friend std::ostream& operator<<(std::ostream& s, const NormalizedIntensityMap& m) {
        return s << m._intensities;
    }

    cv::Mat _intensities;
};

}

}