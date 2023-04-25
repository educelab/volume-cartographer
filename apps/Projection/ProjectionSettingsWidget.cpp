#include "ProjectionSettingsWidget.hpp"

ProjectionSettingsWidget::ProjectionSettingsWidget(
    volcart::ProjectionSettings* projectionSettings)
    : projectionSettings_(projectionSettings)
    , sliceLabel_(new QLabel)
    , sliceSlider_(new QSlider(Qt::Horizontal))
    , colorLabel_(new QLabel("Color"))
    , colorComboBox_(new QComboBox)
    , thicknessLabel_(new QLabel("Thickness"))
    , thicknessSpinBox_(new QSpinBox)
    , intersectOnlyCheckBox_(new QCheckBox("Intersect Only"))
    , layout_(new QVBoxLayout)
{
    sliceLabel_->setMinimumWidth(10);
    sliceSlider_->setMinimum(projectionSettings_->zMin);
    sliceSlider_->setMaximum(projectionSettings_->zMax);
    sliceSlider_->setValue(projectionSettings_->zMin);
    connect(
        sliceSlider_, &QSlider::valueChanged, this,
        &ProjectionSettingsWidget::handleSliceChange);

    colorLabel_->setMinimumWidth(10);
    colorComboBox_->addItems({"White", "Red", "Green", "Blue"});
    if (projectionSettings_->color == WHITE) {
        colorComboBox_->setCurrentText("White");
    } else if (projectionSettings_->color == RED) {
        colorComboBox_->setCurrentText("Red");
    } else if (projectionSettings_->color == GREEN) {
        colorComboBox_->setCurrentText("Green");
    } else if (projectionSettings_->color == BLUE) {
        colorComboBox_->setCurrentText("Blue");
    }
    colorComboBox_->setMinimumWidth(10);
    connect(
        colorComboBox_, &QComboBox::currentTextChanged, this,
        &ProjectionSettingsWidget::handle_settings_change_);

    thicknessLabel_->setMinimumWidth(10);
    thicknessSpinBox_->setMinimum(1);
    thicknessSpinBox_->setValue(projectionSettings_->thickness);
    thicknessSpinBox_->setMinimumWidth(10);
    connect(
        thicknessSpinBox_, &QSpinBox::textChanged, this,
        &ProjectionSettingsWidget::handle_settings_change_);

    intersectOnlyCheckBox_->setChecked(projectionSettings_->intersectOnly);
    intersectOnlyCheckBox_->setMinimumWidth(10);
    connect(
        intersectOnlyCheckBox_, &QCheckBox::stateChanged, this,
        &ProjectionSettingsWidget::handle_settings_change_);

    layout_->addWidget(sliceLabel_);
    layout_->addWidget(sliceSlider_);
    layout_->addWidget(colorLabel_);
    layout_->addWidget(colorComboBox_);
    layout_->addWidget(thicknessLabel_);
    layout_->addWidget(thicknessSpinBox_);
    layout_->addWidget(intersectOnlyCheckBox_);
    layout_->addStretch();
    setLayout(layout_);

    handleSliceChange();
}

void ProjectionSettingsWidget::handleSettingsRequest()
{
    emit settingsChanged(*projectionSettings_, sliceSlider_->value());
}

void ProjectionSettingsWidget::handleSliceChange()
{
    sliceLabel_->setText("Slice: " + QString::number(sliceSlider_->value()));
    emit sliceChanged(sliceSlider_->value());
}

void ProjectionSettingsWidget::handle_settings_change_()
{
    switch (Color(colorComboBox_->currentIndex())) {
        case Color::White:
            projectionSettings_->color = WHITE;
            break;
        case Color::Red:
            projectionSettings_->color = RED;
            break;
        case Color::Green:
            projectionSettings_->color = GREEN;
            break;
        case Color::Blue:
            projectionSettings_->color = BLUE;
            break;
    }
    projectionSettings_->thickness = thicknessSpinBox_->value();
    projectionSettings_->intersectOnly = intersectOnlyCheckBox_->isChecked();

    emit settingsChanged(*projectionSettings_, sliceSlider_->value());
}
