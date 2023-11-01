// SettingsDialog.cpp
// Philip Allgaier 2023 October
#include "SettingsDialog.hpp"

#include <QSettings>
#include <QMessageBox>
#include <QToolTip>

using namespace ChaoVis;

SettingsDialog::SettingsDialog(QWidget *parent) : QDialog(parent)
{
    setupUi(this);

    QSettings settings("VC.ini", QSettings::IniFormat);

    edtDefaultPathVolpkg->setText(settings.value("volpkg/default_path").toString());
    chkAutoOpenVolpkg->setChecked(settings.value("volpkg/auto_open", false).toInt() != 0);

    spinFwdBackStepMs->setValue(settings.value("viewer/fwd_back_step_ms", 25).toInt());
    chkCenterOnZoom->setChecked(settings.value("viewer/center_on_zoom", false).toInt() != 0);
    edtImpactRange->setText(settings.value("viewer/impact_range_steps", "1-20").toString());

    spinPreloadedSlices->setValue(settings.value("perf/preloaded_slices", 200).toInt());

    connect(btnHelpPreloadedSlices, &QPushButton::clicked, this, [this]{ QToolTip::showText(QCursor::pos(), btnHelpPreloadedSlices->toolTip()); });
}

void SettingsDialog::accept()
{
    QSettings settings("VC.ini", QSettings::IniFormat);

    settings.setValue("volpkg/default_path", edtDefaultPathVolpkg->text());
    settings.setValue("volpkg/auto_open", chkAutoOpenVolpkg->isChecked() ? "1" : "0");

    settings.setValue("viewer/fwd_back_step_ms", spinFwdBackStepMs->value());
    settings.setValue("viewer/center_on_zoom", chkCenterOnZoom->isChecked() ? "1" : "0");
    settings.setValue("viewer/impact_range_steps", edtImpactRange->text());

    settings.setValue("perf/preloaded_slices", spinPreloadedSlices->value());

    QMessageBox::information(this, tr("Restart required"), tr("Note: Some settings only take effect once you restarted the app."));

    close();
}
