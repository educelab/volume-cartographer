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
    chkAutoOpenVolpkg->setChecked(settings.value("volpkg/auto_open", true).toInt() != 0);

    spinFwdBackStepMs->setValue(settings.value("viewer/fwd_back_step_ms", 25).toInt());
    chkCenterOnZoom->setChecked(settings.value("viewer/center_on_zoom", false).toInt() != 0);
    edtImpactRange->setText(settings.value("viewer/impact_range_steps", "1-3, 5, 8, 11, 15, 20, 28, 40, 60, 100, 200").toString());
    edtScanRange->setText(settings.value("viewer/scan_range_steps", "1, 2, 5, 10, 20, 50, 100, 200, 500, 1000").toString());
    spinScrollSpeed->setValue(settings.value("viewer/scroll_speed", -1).toInt());
    spinDisplayOpacity->setValue(settings.value("viewer/display_segment_opacity", 70).toInt());
    chkPlaySoundAfterSegRun->setChecked(settings.value("viewer/play_sound_after_seg_run", true).toInt() != 0);

    spinPreloadedSlices->setValue(settings.value("perf/preloaded_slices", 200).toInt());
    chkSkipImageFormatConvExp->setChecked(settings.value("perf/chkSkipImageFormatConvExp", false).toBool());

    connect(btnHelpScrollSpeed, &QPushButton::clicked, this, [this]{ QToolTip::showText(QCursor::pos(), btnHelpScrollSpeed->toolTip()); });
    connect(btnHelpDisplayOpacity, &QPushButton::clicked, this, [this]{ QToolTip::showText(QCursor::pos(), btnHelpDisplayOpacity->toolTip()); });
    connect(btnHelpPreloadedSlices, &QPushButton::clicked, this, [this]{ QToolTip::showText(QCursor::pos(), btnHelpPreloadedSlices->toolTip()); });
}

void SettingsDialog::accept()
{
    // Store the settings
    QSettings settings("VC.ini", QSettings::IniFormat);

    settings.setValue("volpkg/default_path", edtDefaultPathVolpkg->text());
    settings.setValue("volpkg/auto_open", chkAutoOpenVolpkg->isChecked() ? "1" : "0");

    settings.setValue("viewer/fwd_back_step_ms", spinFwdBackStepMs->value());
    settings.setValue("viewer/center_on_zoom", chkCenterOnZoom->isChecked() ? "1" : "0");
    settings.setValue("viewer/impact_range_steps", edtImpactRange->text());
    settings.setValue("viewer/scan_range_steps", edtScanRange->text());
    settings.setValue("viewer/scroll_speed", spinScrollSpeed->value());
    settings.setValue("viewer/display_segment_opacity", spinDisplayOpacity->value());
    settings.setValue("viewer/play_sound_after_seg_run", chkPlaySoundAfterSegRun->isChecked() ? "1" : "0");

    settings.setValue("perf/preloaded_slices", spinPreloadedSlices->value());
    settings.setValue("perf/chkSkipImageFormatConvExp", chkSkipImageFormatConvExp->isChecked() ? "1" : "0");

    QMessageBox::information(this, tr("Restart required"), tr("Note: Some settings only take effect once you restarted the app."));

    close();
}

// Expand string that contains a range definition from the user settings into an integer vector
std::vector<int> SettingsDialog::expandSettingToIntRange(const QString& setting)
{
    std::vector<int> res;
    if (setting.isEmpty()) {
        return res;
    }

    auto value = setting.simplified();
    value.replace(" ", "");
    auto commaSplit = value.split(",");
    for(auto str : commaSplit) {
        if (str.contains("-")) {
            // Expand the range to distinct values
            auto dashSplit = str.split("-");
            // We need to have two split results (before and after the dash), otherwise skip
            if (dashSplit.size() == 2) {
                for(int i = dashSplit.at(0).toInt(); i <= dashSplit.at(1).toInt(); i++) {
                    res.push_back(i);
                }
            }
        } else {
            res.push_back(str.toInt());
        }
    }

    return res;
}