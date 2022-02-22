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

#include "FetchSliceThread.hpp"
#include "PPMProjectionViewerWidget.hpp"
#include "Projection.hpp"
#include "ProjectionSettingsWidget.hpp"
#include "SliceProjectionViewerWidget.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/util/ImageConversion.hpp"

class ProjectionViewerWindow : public QMainWindow
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    explicit ProjectionViewerWindow(
        volcart::ProjectionSettings* settings,
        const vtkSmartPointer<vtkPlane>& cutPlane,
        const vtkSmartPointer<vtkStripper>& stripper,
        volcart::Volume::Pointer& volume,
        QWidget* parent = nullptr);

private:
    SliceProjectionViewerWidget* sliceProjectionViewerWidget_;
    ProjectionSettingsWidget* projectionSettingsWidget_;
    PPMProjectionViewerWidget* ppmProjectionViewerWidget_;
    QSplitter* sidePanelSplitter_;
    QSplitter* mainSplitter_;
};

#pragma clang diagnostic pop
