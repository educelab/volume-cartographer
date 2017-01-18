// CSimpleNumEditBox.cpp
// Chao Du 2015 March
#include "CSimpleNumEditBox.hpp"
#include <QtCore>
#include <QtWidgets>

using namespace ChaoVis;

// Constructor
CSimpleNumEditBox::CSimpleNumEditBox(QWidget* parent)
    : QTextEdit(parent), fImageIndex(0)
{
    setText(QString("%1").arg(fImageIndex));
    setInputMethodHints(Qt::ImhDigitsOnly);
    setWordWrapMode(QTextOption::NoWrap);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    QFontMetrics fm(font());
    int h = qMax(fm.height(), 14) + 4;
    int w = fm.width(QLatin1Char('x')) * 17 + 4;
    QStyleOptionFrame opt;
    opt.initFrom(this);

    setFixedHeight(style()
                       ->sizeFromContents(
                           QStyle::CT_LineEdit, &opt,
                           QSize(w, h).expandedTo(QApplication::globalStrut()),
                           this)
                       .height());
}

// Destructor
CSimpleNumEditBox::~CSimpleNumEditBox(void) {}

// Get slice index from text
int CSimpleNumEditBox::GetImageIndexFromText(void)
{
    int aResult;
    bool aIsOk = false;  // invalid
    aResult = toPlainText().toInt(&aIsOk);
    if (!aIsOk) {
        return -1;
    } else {
        return aResult;
    }
}

// Get slice index
int CSimpleNumEditBox::GetImageIndex(void) { return fImageIndex; }

// Set slice index
void CSimpleNumEditBox::SetImageIndex(int nImageIndex)
{
    fImageIndex = nImageIndex;
    setText(QString("%1").arg(fImageIndex));
}

// Handle key press event
void CSimpleNumEditBox::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return) {
        //        QMessageBox::information( this, tr( "message" ), tr( "key
        //        press detected in edit box" ) );
        int aNewIndex = GetImageIndexFromText();
        if (aNewIndex != fImageIndex) {
            fImageIndex = aNewIndex;
            emit SendSignalOnTextChanged();
        }
    } else {
        QTextEdit::keyPressEvent(event);
    }
}
