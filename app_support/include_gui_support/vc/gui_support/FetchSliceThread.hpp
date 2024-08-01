#pragma once

#include <QMutex>
#include <QThread>
#include <QWaitCondition>

#include "vc/core/types/Volume.hpp"

namespace volcart::gui
{

class FetchSliceThread : public QThread
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    explicit FetchSliceThread(
        Volume::Pointer volume, QObject* parent = nullptr);
    ~FetchSliceThread() override;

    void fetchSlice(int sliceIdx);

signals:
    void fetchedSlice(const cv::Mat& mat);

protected:
    void run() override;

private:
    QMutex mutex_;
    QWaitCondition condition_;
    bool restart_ = false;
    bool abort_ = false;
    Volume::Pointer volume_;
    int sliceIdx_{0};
};
}  // namespace volcart::gui