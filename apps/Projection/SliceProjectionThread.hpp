#pragma once

#include <QMutex>
#include <QThread>
#include <QWaitCondition>
#include <opencv2/core.hpp>
#include <vtkPlane.h>
#include <vtkSmartPointer.h>
#include <vtkStripper.h>

#include "Projection.hpp"

class SliceProjectionThread : public QThread
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    explicit SliceProjectionThread(
        vtkSmartPointer<vtkPlane> cutPlane,
        vtkSmartPointer<vtkStripper> stripper,
        QObject* parent = nullptr);
    ~SliceProjectionThread() override;

    void runProjection(
        cv::Mat& mat, volcart::ProjectionSettings settings, int sliceIdx);

signals:
    void ranProjection(const cv::Mat& pixmap);

protected:
    void run() override;

private:
    QMutex mutex_;
    QWaitCondition condition_;
    bool restart_ = false;
    bool abort_ = false;

    cv::Mat mat_;
    volcart::ProjectionSettings settings_;
    int sliceIdx_;
    vtkSmartPointer<vtkPlane> cutPlane_;
    vtkSmartPointer<vtkStripper> stripper_;
};
