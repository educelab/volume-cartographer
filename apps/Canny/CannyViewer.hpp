#pragma once

#include <QCheckBox>
#include <QLabel>
#include <QMainWindow>
#include <QMutex>
#include <QPixmap>
#include <QScrollArea>
#include <QScrollBar>
#include <QSlider>
#include <QSplitter>
#include <QThread>
#include <QVBoxLayout>
#include <QWaitCondition>
#include <QWheelEvent>

#include "vc/core/types/Volume.hpp"
#include "vc/core/util/Canny.hpp"
#include "vc/core/util/ImageConversion.hpp"

class FetchSliceThread : public QThread
{
    Q_OBJECT  // NOLINT

        public
        : explicit FetchSliceThread(
              volcart::Volume::Pointer volume, QObject* parent = nullptr);
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
    volcart::Volume::Pointer volume_;
    int sliceIdx_;
};

class CannyThread : public QThread
{
    Q_OBJECT  // NOLINT

        public : explicit CannyThread(QObject* parent = nullptr);
    ~CannyThread() override;

    void runCanny(cv::Mat& mat, volcart::CannySettings settings);

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
    volcart::CannySettings settings_;
};

class CannyViewer : public QMainWindow
{
    Q_OBJECT  // NOLINT

        public :

        explicit CannyViewer(
            volcart::CannySettings* settings,
            const volcart::Volume::Pointer& volume,
            QWidget* parent = nullptr);

    auto eventFilter(QObject* obj, QEvent* event) -> bool override;

private slots:
    void update_slice_image_(const cv::Mat& mat);
    void update_pixmap_(const cv::Mat& mat);

private:
    FetchSliceThread fetchSliceThread_;
    CannyThread cannyThread_;

    volcart::CannySettings* settings_;

    int numSlices_;
    double scaleFactor_;

    QLabel* imageLabel_;
    QScrollArea* scrollArea_;
    cv::Mat originalSliceMat_;

    QLabel* blurSizeLabel_;
    QLabel* minThresholdLabel_;
    QLabel* maxThresholdLabel_;
    QLabel* apertureSizeLabel_;
    QLabel* sliceLabel_;
    QLabel* closingSizeLabel_;

    QSlider* blurSizeSlider_;
    QSlider* minThresholdSlider_;
    QSlider* maxThresholdSlider_;
    QSlider* apertureSizeSlider_;
    QSlider* sliceSlider_;
    QCheckBox* contourCheckBox_;
    QSlider* closingSizeSlider_;
    QCheckBox* bilateralCheckBox_;

    QVBoxLayout* sliderLayout_;
    QWidget* sliderWidget_;

    QSplitter* splitter_;

    void handle_slice_change_();
    void handle_settings_change_();
    void wheelEvent(QWheelEvent* event) override;
    void zoom_in_();
    void zoom_out_();
    void scale_image_(double factor);
    static void AdjustScrollBar(QScrollBar* scrollBar, double factor);
};
