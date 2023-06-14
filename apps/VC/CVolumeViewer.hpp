// CVolumeViewer.h
// Chao Du 2015 April
#pragma once

#include <QtWidgets>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "CSimpleNumEditBox.hpp"

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
    bool eventFilter(QObject* watched, QEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void paintEvent(QPaintEvent* event);

public slots:
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
    void ScrollToCenter(cv::Vec2f pos);
    cv::Vec2f GetScrollPosition() const;
    cv::Vec2f CleanScrollPosition(cv::Vec2f pos) const;

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
