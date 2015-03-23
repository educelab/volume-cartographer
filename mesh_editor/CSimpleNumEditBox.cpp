// CSimpleNumEditBox.cpp
// Chao Du 2015 March
#include "CSimpleNumEditBox.h"

#include <QtCore>
#include <QtGui>

using namespace ChaoVis;

// Constructor
CSimpleNumEditBox::CSimpleNumEditBox( QWidget *parent ) :
    QTextEdit( parent ),
    fSliceIndex( 0 )
{
    setText( QString( "%1" ).arg( fSliceIndex ) );
    setInputMethodHints( Qt::ImhDigitsOnly );
    setWordWrapMode( QTextOption::NoWrap );
    setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
    setVerticalScrollBarPolicy( Qt::ScrollBarAlwaysOff );

    QFontMetrics fm( font() );
    int h = qMax( fm.height(), 14 ) + 4;
    int w = fm.width( QLatin1Char( 'x' ) ) * 17 + 4;
    QStyleOptionFrameV2 opt;
    opt.initFrom( this );

    setFixedHeight( style()->sizeFromContents( QStyle::CT_LineEdit, &opt, QSize( w, h ).expandedTo( QApplication::globalStrut()), this ).height() );
}

// Destructor
CSimpleNumEditBox::~CSimpleNumEditBox( void )
{
}

// Get slice index from text
int CSimpleNumEditBox::GetSliceIndexFromText( void )
{
    int aResult;
    bool aIsOk = false; // invalid
    aResult = toPlainText().toInt( &aIsOk );
    if ( !aIsOk ) {
        return -1;
    } else {
        return aResult;
    }
}

// Get slice index
int CSimpleNumEditBox::GetSliceIndex( void )
{
    return fSliceIndex;
}

// Set slice index
void CSimpleNumEditBox::SetSliceIndex( int nSliceIndex )
{
    fSliceIndex = nSliceIndex;
    setText( QString( "%1" ).arg( fSliceIndex ) );
}

// Handle key press event
void CSimpleNumEditBox::keyPressEvent( QKeyEvent *event )
{
    if ( event->key() == Qt::Key_Enter ) {
//        QMessageBox::information( this, tr( "message" ), tr( "key press detected in edit box" ) );
        int aNewIndex = GetSliceIndexFromText();
        if ( aNewIndex != fSliceIndex ) {
            fSliceIndex = aNewIndex;
            emit SendSignalOnTextChanged();
        }
    } else {
        QTextEdit::keyPressEvent( event );
    }
}
