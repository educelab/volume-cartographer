#include "CannyThread.hpp"

#include <utility>

#include <opencv2/imgproc.hpp>

#include "vc/core/util/ImageConversion.hpp"

namespace
{
auto Percentile(const cv::Mat& a, const float perc) -> float
{
    const int histSize{256};
    float range[] = {0, 256};
    const float* histRange[] = {range};
    const int numPixs = a.rows * a.cols;
    const float pixReq = static_cast<float>(numPixs) * perc;
    cv::Mat hist;

    cv::calcHist(
        &a, 1, nullptr, cv::Mat(), hist, 1, &histSize, histRange, true, false);

    float cnt{0};
    for (int i = 0; i < histSize; i++) {
        cnt += hist.at<float>(i);
        if (cnt >= pixReq) {
            return static_cast<float>(i);
        }
    }
    return 255.f;
}

auto Window(const cv::Mat& a, const float low, const float high)
{
    cv::Mat b;
    a.convertTo(
        b, a.depth(), 255.f / (high - low), -low * 255.f / (high - low));
    return b;
}
}  // namespace

CannyThread::CannyThread(QObject* parent) : QThread(parent) {}

CannyThread::~CannyThread()
{
    mutex_.lock();
    abort_ = true;
    condition_.wakeOne();
    mutex_.unlock();
    wait();
}

void CannyThread::runCanny(cv::Mat& mat, CannySettings settings)
{
    QMutexLocker locker(&mutex_);

    if (!mat.empty()) {
        mat_ = mat.clone();
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
        cv::Mat src = mat_;
        const CannySettings settings = settings_;
        mutex_.unlock();

        if (!restart_) {
            cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
            // Get the canny edges
            cv::Mat dst = volcart::Canny(src, settings);
            // Run normalize
            if (settings.normalize) {
                const auto low = Percentile(src, 0.01f);
                const auto high = Percentile(src, 0.99f);
                src = Window(src, low, high);
            }
            // Draw them on the slice
            src = volcart::ColorConvertImage(src, dst.channels());
            cv::addWeighted(src, 0.5, dst, 0.5, 0, dst);
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