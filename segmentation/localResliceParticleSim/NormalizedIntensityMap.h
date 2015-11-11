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

    void draw(const int32_t=100, const int32_t=100) const;

    std::vector<std::pair<int32_t, double>> findMaxima(int32_t index) const;

private:
    friend std::ostream& operator<<(std::ostream& s, const NormalizedIntensityMap& m) {
        return s << m._intensities;
    }

    cv::Mat _intensities;
};

}

}
