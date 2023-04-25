#pragma once

#include <QMutex>
#include <QThread>
#include <QWaitCondition>
#include <opencv2/core.hpp>

class PPMProjectionThread : public QThread
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    explicit PPMProjectionThread(
        const std::string& visualizePPMIntersection,
        const std::string& ppmImageOverlay);
    ~PPMProjectionThread() override;

    void runProjection(int sliceIdx);

signals:
    void ranProjection(const cv::Mat& pixmap);

protected:
    void run() override;

private:
    QMutex mutex_;
    QWaitCondition condition_;
    bool restart_ = false;
    bool abort_ = false;

    int sliceIdx_;

    cv::Mat originalPPMImage_;
    cv::Mat ppmMask_;
    cv::Mat zChannelImage_;
};
