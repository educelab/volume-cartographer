// CSimpleNumEditBox.h
// Chao Du 2015 March
#pragma once

#include <QTextEdit>

namespace ChaoVis {

class CSimpleNumEditBox : public QTextEdit {

    Q_OBJECT

public:
    CSimpleNumEditBox( QWidget *parent = 0 );
    ~CSimpleNumEditBox( void );

    int GetImageIndexFromText( void ) const;
    int GetImageIndex( void ) const;
    void SetImageIndex( int nImageIndex );

protected:
    void keyPressEvent( QKeyEvent *event );

private slots:

signals:
    void SendSignalOnTextChanged( void );

private:
    int fImageIndex;

}; // class CSimpleNumEditBox

} // namespace ChaoVis
