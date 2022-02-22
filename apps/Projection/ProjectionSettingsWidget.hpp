#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>

#include "Projection.hpp"

class ProjectionSettingsWidget : public QWidget
{
    // clang-format off
    Q_OBJECT
    // clang-format on

public:
    explicit ProjectionSettingsWidget(
        volcart::ProjectionSettings* projectionSettings);

public slots:
    void handleSliceChange();
    void handleSettingsRequest();

signals:
    void settingsChanged(volcart::ProjectionSettings settings, int sliceIdx);
    void sliceChanged(int sliceIdx);

private:
    volcart::ProjectionSettings* projectionSettings_;

    QLabel* sliceLabel_;
    QSlider* sliceSlider_;
    QLabel* colorLabel_;
    QComboBox* colorComboBox_;
    QLabel* thicknessLabel_;
    QSpinBox* thicknessSpinBox_;
    QCheckBox* intersectOnlyCheckBox_;

    QVBoxLayout* layout_;

private slots:
    void handle_settings_change_();
};
