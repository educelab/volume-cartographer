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

#include "CannyThread.hpp"
#include "SliceCannyViewerWidget.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/util/Canny.hpp"
#include "vc/core/util/ImageConversion.hpp"
#include "vc/gui_support/FetchSliceThread.hpp"

class CannyViewerWindow : public QMainWindow
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    explicit CannyViewerWindow(
        volcart::CannySettings* settings,
        const volcart::Volume::Pointer& volume,
        QWidget* parent = nullptr);

public slots:
    void handleSettingsRequest();

private:
    SliceCannyViewerWidget* sliceCannyViewerWidget_;

    volcart::CannySettings* settings_;

    int numSlices_;

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

    void handle_settings_change_();
    void handle_slice_change_();
};
