// SettingsDialog.cpp
// Philip Allgaier 2023 October
#include "SettingsDialog.hpp"

#include <QSettings>
#include <QMessageBox>

using namespace ChaoVis;

SettingsDialog::SettingsDialog(QWidget *parent) : QDialog(parent)
{
    setupUi(this);

    QSettings settings("VC.ini", QSettings::IniFormat);

    edtDefaultPathVolpkg->setText(settings.value("volpkg/default_path").toString());
    chkAutoOpenVolpkg->setChecked(settings.value("volpkg/auto_open", false).toInt() != 0);

    spinFwdBackStepMs->setValue(settings.value("viewer/fwd_back_step_ms", 25).toInt());
    chkCenterOnZoom->setChecked(settings.value("viewer/center_on_zoom", false).toInt() != 0);
}

void SettingsDialog::accept()
{
    QSettings settings("VC.ini", QSettings::IniFormat);

    settings.setValue("volpkg/default_path", edtDefaultPathVolpkg->text());
    settings.setValue("volpkg/auto_open", chkAutoOpenVolpkg->isChecked() ? "1" : "0");

    settings.setValue("viewer/fwd_back_step_ms", spinFwdBackStepMs->value());
    settings.setValue("viewer/center_on_zoom", chkCenterOnZoom->isChecked() ? "1" : "0");

    QMessageBox::information(this, tr("Restart required"), tr("Note: Some settings only take effect once you restarted the app."));

    close();
}
