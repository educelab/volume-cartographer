// CSimpleNumEditBox.h
// Chao Du 2015 March
#ifndef _CSIMPLENUMEDITBOX_H_
#define _CSIMPLENUMEDITBOX_H_

#include <QTextEdit>

namespace ChaoVis {

class CSimpleNumEditBox : public QTextEdit {

    Q_OBJECT

public:
    CSimpleNumEditBox( QWidget *parent = 0 );
    ~CSimpleNumEditBox( void );

    int GetImageIndexFromText( void );
    int GetImageIndex( void );
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

#endif // _CSIMPLENUMEDITBOX_H_
