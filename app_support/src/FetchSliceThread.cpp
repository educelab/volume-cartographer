#include "vc/gui_support/FetchSliceThread.hpp"

#include "vc/core/util/ImageConversion.hpp"

namespace vcg = volcart::gui;

vcg::FetchSliceThread::FetchSliceThread(
    volcart::Volume::Pointer volume, QObject* parent)
    : QThread(parent), volume_(std::move(volume))
{
}

vcg::FetchSliceThread::~FetchSliceThread()
{
    mutex_.lock();
    abort_ = true;
    condition_.wakeOne();
    mutex_.unlock();
    wait();
}

void vcg::FetchSliceThread::fetchSlice(int sliceIdx)
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

void vcg::FetchSliceThread::run()
{
    forever
    {
        if (abort_) {
            return;
        }

        mutex_.lock();
        const int sliceIdx = sliceIdx_;
        mutex_.unlock();

        if (!restart_) {
            const auto slice = volume_->getSliceDataCopy(sliceIdx);
            auto src = volcart::QuantizeImage(slice, CV_8U, false);
            emit fetchedSlice(src);
        }

        mutex_.lock();
        if (!restart_) {
            condition_.wait(&mutex_);
        }
        restart_ = false;
        mutex_.unlock();
    }
}
