#pragma clang diagnostic push
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "readability-redundant-access-specifiers"
#pragma ide diagnostic ignored "cppcoreguidelines-special-member-functions"
#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QMainWindow>
#include <QMutex>
#include <QPixmap>
#include <QScrollArea>
#include <QScrollBar>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QThread>
#include <QVBoxLayout>
#include <QWaitCondition>
#include <QWheelEvent>
#include <vtkCutter.h>
#include <vtkPlane.h>
#include <vtkSmartPointer.h>
#include <vtkStripper.h>

#include "Projection.hpp"
#include "vc/core/types/Volume.hpp"
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

class ProjectionThread : public QThread
{
    Q_OBJECT  // NOLINT

        public : explicit ProjectionThread(
                     vtkSmartPointer<vtkPlane> cutPlane,
                     vtkSmartPointer<vtkStripper> stripper,
                     QObject* parent = nullptr);
    ~ProjectionThread() override;

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

class ProjectionViewer : public QMainWindow
{
    Q_OBJECT  // NOLINT

        public :

        explicit ProjectionViewer(
            volcart::ProjectionSettings* settings,
            vtkSmartPointer<vtkPlane> cutPlane,
            vtkSmartPointer<vtkStripper> stripper,
            const volcart::Volume::Pointer& volume,
            QWidget* parent = nullptr);

    auto eventFilter(QObject* obj, QEvent* event) -> bool override;

private slots:
    void update_slice_image_(const cv::Mat& mat);
    void update_pixmap_(const cv::Mat& mat);

private:
    FetchSliceThread fetchSliceThread_;
    ProjectionThread projectionThread_;

    volcart::ProjectionSettings* settings_;

    double scaleFactor_;

    QLabel* imageLabel_;
    QScrollArea* scrollArea_;
    cv::Mat originalSliceMat_;

    QLabel* sliceLabel_;
    QSlider* sliceSlider_;
    QLabel* colorLabel_;
    QComboBox* colorComboBox_;
    QLabel* thicknessLabel_;
    QSpinBox* thicknessSpinBox_;
    QCheckBox* intersectOnlyCheckBox_;

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

#pragma clang diagnostic pop
