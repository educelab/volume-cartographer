#pragma once

#include <QMutex>
#include <QThread>
#include <QWaitCondition>
#include <opencv2/core.hpp>

#include "vc/core/util/Canny.hpp"

struct CannySettings : volcart::CannySettings {
    bool normalize{false};
};

class CannyThread : public QThread
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    explicit CannyThread(QObject* parent = nullptr);
    ~CannyThread() override;

    void runCanny(cv::Mat& mat, CannySettings settings);

signals:
    void ranCanny(const cv::Mat& pixmap);

protected:
    void run() override;

private:
    QMutex mutex_;
    QWaitCondition condition_;
    bool restart_ = false;
    bool abort_ = false;

    cv::Mat mat_;
    CannySettings settings_;
};