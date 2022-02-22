#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"

#include <QGuiApplication>
#include <QScreen>

#include "ProjectionViewerWindow.hpp"

ProjectionViewerWindow::ProjectionViewerWindow(
    volcart::ProjectionSettings* settings,
    const vtkSmartPointer<vtkPlane>& cutPlane,
    const vtkSmartPointer<vtkStripper>& stripper,
    volcart::Volume::Pointer& volume,
    QWidget* parent)
    : QMainWindow(parent)
    , mainSplitter_(new QSplitter)
    , sidePanelSplitter_(new QSplitter(Qt::Vertical))
    , sliceProjectionViewerWidget_(
          new SliceProjectionViewerWidget(volume, cutPlane, stripper))
    , projectionSettingsWidget_(new ProjectionSettingsWidget(settings))
    , ppmProjectionViewerWidget_(new PPMProjectionViewerWidget(
          settings->visualizePPMIntersection, settings->ppmImageOverlay))
{
    connect(
        projectionSettingsWidget_, &ProjectionSettingsWidget::sliceChanged,
        sliceProjectionViewerWidget_,
        &SliceProjectionViewerWidget::handleSliceChange);
    connect(
        projectionSettingsWidget_, &ProjectionSettingsWidget::settingsChanged,
        sliceProjectionViewerWidget_,
        &SliceProjectionViewerWidget::handleSettingsChange);
    connect(
        sliceProjectionViewerWidget_, &SliceProjectionViewerWidget::sliceLoaded,
        projectionSettingsWidget_,
        &ProjectionSettingsWidget::handleSettingsRequest);
    connect(
        projectionSettingsWidget_, &ProjectionSettingsWidget::sliceChanged,
        ppmProjectionViewerWidget_,
        &PPMProjectionViewerWidget::handleSliceChange);
    projectionSettingsWidget_->handleSliceChange();

    sidePanelSplitter_->addWidget(projectionSettingsWidget_);
    sidePanelSplitter_->addWidget(ppmProjectionViewerWidget_);
    sidePanelSplitter_->setSizes({300, 300});

    mainSplitter_->addWidget(sliceProjectionViewerWidget_);
    mainSplitter_->addWidget(sidePanelSplitter_);
    mainSplitter_->setSizes({2, 1});

    setCentralWidget(mainSplitter_);

    const float sizeRatio = 3.0 / 5.0;
    resize(QGuiApplication::primaryScreen()->availableSize() * sizeRatio);
}

#pragma clang diagnostic pop
