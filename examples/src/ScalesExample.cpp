//
// Created by Seth Parker on 4/25/17.
//

#include "vc/core/scales/Scales.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

int main()
{
    cv::Mat micro(23, 83, CV_16UC1, g_scaleMicro);
    cv::Mat pi(76, 94, CV_16UC1, g_scalePi);
    cv::Mat small(27, 254, CV_16UC1, g_scaleSmall);

    micro *= 255;
    pi *= 255;
    small *= 255;

    cv::imwrite("micro.png", micro);
    cv::imwrite("pi.png", pi);
    cv::imwrite("small.png", small);
}