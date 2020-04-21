#pragma once

#include <QDialog>
#include <QKeyEvent>

class BlockingDialog : public QDialog
{
    Q_OBJECT
public:
    explicit BlockingDialog(
        QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags())
        : QDialog(parent, f)
    {
        setModal(true);
        setWindowFlags(Qt::Window | Qt::CustomizeWindowHint);
    }

    void keyPressEvent(QKeyEvent* e) override
    {
        if (e->key() == Qt::Key_Escape) {
            // pass
            e->accept();
        } else {
            QDialog::keyPressEvent(e);
        }
    }
};
