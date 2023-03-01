#pragma once

#include <QColorDialog>
#include <QLabel>

class ColorFrame : public QLabel
{
    Q_OBJECT
public:
    explicit ColorFrame(
        QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags())
        : QLabel(parent, f)
    {
        setMinimumSize(4, 4);
        setFrameShape(QFrame::StyledPanel);
        setFrameShadow(QFrame::Plain);
        setColor(QColor("blue"));
    }

    [[nodiscard]] auto color() const -> QColor { return color_; }

public slots:
    void setColor(QColor color)
    {
        if (color_ != color) {
            color_ = color;
            auto style =
                QString("QLabel { background-color: %1 }").arg(color_.name());
            setStyleSheet(style);
            emit colorChanged(color_);
        }
    }

signals:
    void colorChanged(QColor color);

protected:
    void mousePressEvent(QMouseEvent* ev) override { pick_color_(); }

private:
    void pick_color_()
    {
        auto color = QColorDialog::getColor(color_, this, "Select color");
        if (color.isValid()) {
            setColor(color);
        }
    }

    QColor color_;
};