// CVolumeViewer.h
// Chao Du 2015 April
#pragma once

#include <QtWidgets>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QDebug>

namespace ChaoVis
{

class CVolumeViewerView : public QGraphicsView
{

    Q_OBJECT

    public:
        CVolumeViewerView(QWidget* parent = nullptr);

        void keyPressEvent(QKeyEvent* event) override;
        void keyReleaseEvent(QKeyEvent* event) override;

        bool isRangeKeyPressed() { return rangeKeyPressed; }

    protected:
        bool rangeKeyPressed{false};
};

class CVolumeViewer : public QWidget
{

    Q_OBJECT

public:
    QPushButton* fNextBtn;
    QPushButton* fPrevBtn;
    CVolumeViewer(QWidget* parent = 0);
    ~CVolumeViewer(void);
    virtual void setButtonsEnabled(bool state);

    virtual void SetImage(const QImage& nSrc);
    void SetImageIndex(int nImageIndex)
    {
        fImageIndex = nImageIndex;
        fImageIndexEdit->setValue(nImageIndex);
        UpdateButtons();
    }
    void setNumSlices(int num);

protected:
    bool eventFilter(QObject* watched, QEvent* event);
    void paintEvent(QPaintEvent* event);

public slots:
    void OnZoomInClicked(void);
    void OnZoomOutClicked(void);
    void OnResetClicked(void);
    void OnNextClicked(void);
    void OnPrevClicked(void);
    void OnImageIndexEditTextChanged(void);

signals:
    void SendSignalOnNextSliceShift(int shift);
    void SendSignalOnPrevSliceShift(int shift);
    void SendSignalOnLoadAnyImage(int nImageIndex);
    void SendSignalStatusMessageAvailable(QString text, int timeout);
    void SendSignalImpactRangeUp(void);
    void SendSignalImpactRangeDown(void);

protected:
    void ScaleImage(double nFactor);
    void CenterOn(const QPointF& point);
    virtual void UpdateButtons(void);
    void AdjustScrollBar(QScrollBar* nScrollBar, double nFactor);
    void ScrollToCenter(cv::Vec2f pos);
    cv::Vec2f GetScrollPosition() const;
    cv::Vec2f CleanScrollPosition(cv::Vec2f pos) const;

protected:
    // widget components
    CVolumeViewerView* fGraphicsView;
    QGraphicsScene* fScene;

    QLabel* fCanvas;
    QScrollArea* fScrollArea;
    QPushButton* fZoomInBtn;
    QPushButton* fZoomOutBtn;
    QPushButton* fResetBtn;
    QSpinBox* fImageIndexEdit;
    QHBoxLayout* fButtonsLayout;

    // data
    QImage* fImgQImage;
    double fScaleFactor;
    int fImageIndex;

    bool fCenterOnZoomEnabled;

    QGraphicsPixmapItem* fBaseImageItem;
};  // class CVolumeViewer

}  // namespace ChaoVis
