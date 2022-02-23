#include "vc/gui_support/ImageScrollArea.hpp"

#include <QGuiApplication>
#include <QMouseEvent>
#include <QScrollBar>

namespace vcg = volcart::gui;

vcg::ImageScrollArea::ImageScrollArea()
    : scaleFactor_(1.0), imageLabel_(new QLabel)
{
    imageLabel_->setBackgroundRole(QPalette::Base);
    imageLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel_->setScaledContents(true);

    setBackgroundRole(QPalette::Dark);
    setWidget(imageLabel_);
    verticalScrollBar()->installEventFilter(this);
}

void vcg::ImageScrollArea::updatePixmap(const cv::Mat& mat)
{
    bool needsSizeUpdate = false;
    if (imageLabel_->pixmap(Qt::ReturnByValue).isNull() or
        mat.size[1] != imageLabel_->pixmap(Qt::ReturnByValue).size().width() or
        mat.size[0] != imageLabel_->pixmap(Qt::ReturnByValue).size().height()) {
        needsSizeUpdate = true;
    }

    QImage image = QImage(
        mat.data, mat.cols, mat.rows, static_cast<int>(mat.step),
        QImage::Format_BGR888);
    QPixmap pixmap = QPixmap::fromImage(image);
    imageLabel_->setPixmap(pixmap);

    if (needsSizeUpdate) {
        scaleFactor_ =
            static_cast<float>(this->width()) /
            static_cast<float>(imageLabel_->pixmap(Qt::ReturnByValue).width());
        imageLabel_->resize(
            scaleFactor_ * imageLabel_->pixmap(Qt::ReturnByValue).size());
    }
}

void vcg::ImageScrollArea::mousePressEvent(QMouseEvent* event)
{
    this->setCursor(Qt::ClosedHandCursor);
    lastMouseMoveLocation_ = event->pos();
}

void vcg::ImageScrollArea::mouseReleaseEvent(QMouseEvent* /*unused*/)
{
    dragStarted_ = false;
    this->unsetCursor();
}

void vcg::ImageScrollArea::mouseMoveEvent(QMouseEvent* event)
{
    if (dragStarted_) {
        QPoint delta = event->pos() - lastMouseMoveLocation_;
        this->horizontalScrollBar()->setValue(
            this->horizontalScrollBar()->value() - delta.x());
        this->verticalScrollBar()->setValue(
            this->verticalScrollBar()->value() - delta.y());
    }
    dragStarted_ = true;
    lastMouseMoveLocation_ = event->pos();
}

void vcg::ImageScrollArea::wheelEvent(QWheelEvent* event)
{
    if (static_cast<bool>(
            QGuiApplication::keyboardModifiers() & Qt::ControlModifier)) {
        int numDegrees = event->angleDelta().y() / 8;

        if (numDegrees > 0) {
            this->zoom_in_();
        } else if (numDegrees < 0) {
            this->zoom_out_();
        }

        event->accept();
    } else {
        QScrollArea::wheelEvent(event);
    }
}

void vcg::ImageScrollArea::zoom_in_() { scale_image_(1.25); }

void vcg::ImageScrollArea::zoom_out_() { scale_image_(0.8); }

void vcg::ImageScrollArea::scale_image_(double factor)
{
    scaleFactor_ *= factor;
    imageLabel_->resize(
        scaleFactor_ * imageLabel_->pixmap(Qt::ReturnByValue).size());

    AdjustScrollBar(horizontalScrollBar(), factor);
    AdjustScrollBar(verticalScrollBar(), factor);
}

void vcg::ImageScrollArea::AdjustScrollBar(QScrollBar* scrollBar, double factor)
{
    scrollBar->setValue(
        int(factor * scrollBar->value() +
            ((factor - 1) * scrollBar->pageStep() / 2)));
}
