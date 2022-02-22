#pragma once

#include <QLabel>
#include <QScrollArea>
#include <opencv2/core.hpp>

class ImageScrollArea : public QScrollArea
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    ImageScrollArea();

public slots:
    void updatePixmap(const cv::Mat& mat);

private:
    QLabel* imageLabel_;

    double scaleFactor_;

    bool dragStarted_ = false;
    QPoint lastMouseMoveLocation_ = QPoint(0, 0);

    void zoom_in_();
    void zoom_out_();
    void scale_image_(double factor);
    static void AdjustScrollBar(QScrollBar* scrollBar, double factor);

    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* /*unused*/) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
};