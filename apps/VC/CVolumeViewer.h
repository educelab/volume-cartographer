// CVolumeViewer.h
// Chao Du 2015 April
#pragma once

#include "VCNewGuiHeader.h"

#include <QtWidgets>

#include "CSimpleNumEditBox.h"

namespace ChaoVis
{

class CVolumeViewer : public QWidget
{

    Q_OBJECT

public:
    CVolumeViewer(QWidget* parent = 0);
    ~CVolumeViewer(void);
    virtual void setButtonsEnabled(bool state);

    virtual void SetImage(const QImage& nSrc);
    void SetImageIndex(int nImageIndex)
    {
        fImageIndex = nImageIndex;
        UpdateButtons();
    }

protected:
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void paintEvent(QPaintEvent* event);

private slots:
    void OnZoomInClicked(void);
    void OnZoomOutClicked(void);
    void OnResetClicked(void);
    void OnNextClicked(void);
    void OnPrevClicked(void);
    void OnImageIndexEditTextChanged(void);

signals:
    void SendSignalOnNextClicked(void);
    void SendSignalOnPrevClicked(void);
    void SendSignalOnLoadAnyImage(int nImageIndex);

protected:
    void ScaleImage(double nFactor);
    virtual void UpdateButtons(void);
    void AdjustScrollBar(QScrollBar* nScrollBar, double nFactor);

protected:
    // widget components
    QLabel* fCanvas;
    QScrollArea* fScrollArea;
    QPushButton* fZoomInBtn;
    QPushButton* fZoomOutBtn;
    QPushButton* fResetBtn;
    QPushButton* fNextBtn;
    QPushButton* fPrevBtn;
    CSimpleNumEditBox* fImageIndexEdit;
    QHBoxLayout* fButtonsLayout;

    // data
    QImage* fImgQImage;
    double fScaleFactor;
    int fImageIndex;

};  // class CVolumeViewer

}  // namespace ChaoVis
