#include "CannyThread.hpp"

#include <utility>
#include <vc/core/util/ImageConversion.hpp>

CannyThread::CannyThread(QObject* parent) : QThread(parent) {}

CannyThread::~CannyThread()
{
    mutex_.lock();
    abort_ = true;
    condition_.wakeOne();
    mutex_.unlock();
    wait();
}

void CannyThread::runCanny(
    const cv::Mat& mat,
    const cv::Mat& displayMat,
    volcart::CannySettings settings)
{
    QMutexLocker locker(&mutex_);

    if (not mat.empty()) {
        origMat_ = mat.clone();
        displayMat_ = displayMat.clone();
        settings_ = std::move(settings);

        if (!isRunning()) {
            start(LowPriority);
        } else {
            restart_ = true;
            condition_.wakeOne();
        }
    }
}

void CannyThread::run()
{
    forever
    {
        if (abort_) {
            return;
        }

        mutex_.lock();
        const cv::Mat src = origMat_;
        cv::Mat dispSrc = displayMat_;
        const volcart::CannySettings settings = settings_;
        mutex_.unlock();

        if (!restart_) {
            // Get the canny edges
            cv::Mat dst = volcart::Canny(src, settings);
            // Draw them on the slice
            dispSrc = volcart::ColorConvertImage(dispSrc, dst.channels());
            cv::addWeighted(dispSrc, 0.5, dst, 0.5, 0, dst);
            cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
            // Emit that pixmap to be rendered
            emit ranCanny(dst);
        }

        mutex_.lock();
        if (!restart_) {
            condition_.wait(&mutex_);
        }
        restart_ = false;
        mutex_.unlock();
    }
}