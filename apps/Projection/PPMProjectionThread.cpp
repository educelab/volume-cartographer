#include "PPMProjectionThread.hpp"

#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include "vc/core/types/PerPixelMap.hpp"

PPMProjectionThread::PPMProjectionThread(
    const std::string& visualizePPMIntersection,
    const std::string& ppmImageOverlay)
{
    if (!visualizePPMIntersection.empty()) {
        std::cout << "Loading PPM..." << std::endl;
        auto ppm = volcart::PerPixelMap::ReadPPM(visualizePPMIntersection);
        ppmMask_ = ppm.mask();
        std::cout << "Reading Z channel of PPM..." << std::endl;
        zChannelImage_ = cv::Mat::zeros(
            static_cast<int>(ppm.height()), static_cast<int>(ppm.width()),
            CV_64FC1);
        for (const auto& mapping : ppm.getMappings()) {
            zChannelImage_.at<double>(
                static_cast<int>(mapping.y), static_cast<int>(mapping.x)) =
                mapping.pos[2];
        }

        originalPPMImage_ = cv::imread(ppmImageOverlay);
        emit ranProjection(originalPPMImage_);
    }
}

PPMProjectionThread::~PPMProjectionThread()
{
    mutex_.lock();
    abort_ = true;
    condition_.wakeOne();
    mutex_.unlock();
    wait();
}

void PPMProjectionThread::runProjection(int sliceIdx)
{
    QMutexLocker locker(&mutex_);

    sliceIdx_ = sliceIdx;

    if (!isRunning()) {
        start(LowPriority);
    } else {
        restart_ = true;
        condition_.wakeOne();
    }
}

void PPMProjectionThread::run()
{
    forever
    {
        if (abort_) {
            return;
        }

        mutex_.lock();
        auto sliceIdx = sliceIdx_;
        mutex_.unlock();

        if (!restart_) {
            if (!zChannelImage_.empty()) {
                cv::Mat overlay;
                // Compute slice line overlay
                auto thresholdBoundary = 10;
                cv::inRange(
                    zChannelImage_, sliceIdx - thresholdBoundary,
                    sliceIdx + thresholdBoundary, overlay);
                // Mask the boundary line to only within PPM mask
                overlay.convertTo(overlay, CV_8UC1);
                cv::bitwise_and(overlay, ppmMask_, overlay);
                // Get inverse mask for masking background
                cv::Mat inverseMask;
                cv::bitwise_not(overlay, inverseMask);
                // Convert white line to color
                cv::Mat black = cv::Mat(
                    zChannelImage_.rows, zChannelImage_.cols, CV_8UC1, 0.0);
                std::vector<cv::Mat> channels = {black, overlay, black};
                cv::merge(channels, overlay);
                // Get image background
                cv::Mat background;
                cv::bitwise_and(
                    originalPPMImage_, originalPPMImage_, background,
                    inverseMask);
                // Overlay on image
                cv::add(background, overlay, overlay);

                emit ranProjection(overlay);
            }
        }

        mutex_.lock();
        if (!restart_) {
            condition_.wait(&mutex_);
        }
        restart_ = false;
        mutex_.unlock();
    }
}
