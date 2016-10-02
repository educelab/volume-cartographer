// CVolumeViewer.cpp
// Chao Du 2015 April
#include "CVolumeViewer.h"
#include "HBase.h"

using namespace ChaoVis;

// Constructor
CVolumeViewer::CVolumeViewer( QWidget *parent ) :
    QWidget( parent ),
    fImgQImage( nullptr ),
    fCanvas( nullptr ),
    fScrollArea( nullptr ),
    fZoomInBtn( nullptr ),
    fZoomOutBtn( nullptr ),
    fResetBtn( nullptr ),
    fNextBtn( nullptr ),
    fPrevBtn( nullptr ),
    fScaleFactor( 1.0 ),
    fImageIndex( 0 )
{
    // buttons
    fZoomInBtn = new QPushButton( tr( "Zoom In" ), this );
    fZoomOutBtn = new QPushButton( tr( "Zoom Out" ), this );
    fResetBtn = new QPushButton( tr( "Reset" ), this );
    fNextBtn = new QPushButton( tr( "Next Slice" ), this );
    fPrevBtn = new QPushButton( tr( "Previous Slice" ), this );

    // text edit
    fImageIndexEdit = new CSimpleNumEditBox( this );
    fImageIndexEdit->setEnabled( true );
    connect( fImageIndexEdit, SIGNAL( SendSignalOnTextChanged() ), this, SLOT( OnImageIndexEditTextChanged() ) );

    // create image label
    fCanvas = new QLabel;
    fCanvas->setBackgroundRole( QPalette::Base );
    fCanvas->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
    fCanvas->setScaledContents( true );

    // create scroll area
    fScrollArea = new QScrollArea;
    fScrollArea->setBackgroundRole( QPalette::Dark );
    fScrollArea->setWidget( fCanvas );

    fButtonsLayout = new QHBoxLayout;
    fButtonsLayout->addWidget( fZoomInBtn );
    fButtonsLayout->addWidget( fZoomOutBtn );
    fButtonsLayout->addWidget( fResetBtn );
    fButtonsLayout->addWidget( fPrevBtn );
    fButtonsLayout->addWidget( fNextBtn );
    fButtonsLayout->addWidget( fImageIndexEdit );

    connect( fZoomInBtn, SIGNAL( clicked() ), this, SLOT( OnZoomInClicked() ) );
    connect( fZoomOutBtn, SIGNAL( clicked() ), this, SLOT( OnZoomOutClicked() ) );
    connect( fResetBtn, SIGNAL( clicked() ), this, SLOT( OnResetClicked() ) );
    connect( fNextBtn, SIGNAL( clicked() ), this, SLOT( OnNextClicked() ) );
    connect( fPrevBtn, SIGNAL( clicked() ), this, SLOT( OnPrevClicked() ) );

    QVBoxLayout *aWidgetLayout = new QVBoxLayout;
    aWidgetLayout->addWidget( fScrollArea );
    aWidgetLayout->addLayout( fButtonsLayout );

    setLayout( aWidgetLayout );

    UpdateButtons();
}

// Destructor
CVolumeViewer::~CVolumeViewer( void )
{
    deleteNULL( fCanvas );
    deleteNULL( fScrollArea );
    deleteNULL( fZoomInBtn );
    deleteNULL( fZoomOutBtn );
    deleteNULL( fResetBtn );
    deleteNULL( fPrevBtn );
    deleteNULL( fNextBtn );
    deleteNULL( fImageIndexEdit );
}

void CVolumeViewer::setButtonsEnabled( bool state )
{
    fZoomOutBtn->setEnabled(state);
    fZoomInBtn->setEnabled(state);
    fPrevBtn->setEnabled(state);
    fNextBtn->setEnabled(state);
    fImageIndexEdit->setEnabled(state);
}

// Set image
void CVolumeViewer::SetImage( const QImage &nSrc )
{
    if ( fImgQImage == nullptr ) {
        fImgQImage = new QImage( nSrc );
    } else {
        *fImgQImage = nSrc;
    }

    fCanvas->setPixmap( QPixmap::fromImage( *fImgQImage ) );
    fCanvas->resize( fScaleFactor * fCanvas->pixmap()->size() );

    UpdateButtons();
    update();
}

// Handle mouse press event
void CVolumeViewer::mousePressEvent( QMouseEvent *event )
{
    QMessageBox::information( this, tr( "info" ), tr( "mouse pressed inside the volume viewer" ) );
}

// Handle mouse move event
void CVolumeViewer::mouseMoveEvent( QMouseEvent *event )
{
}

// Handle paint event
void CVolumeViewer::paintEvent( QPaintEvent *event )
{
    // REVISIT - FILL ME HERE
}

// Scale image
void CVolumeViewer::ScaleImage( double nFactor )
{
    Q_ASSERT( fCanvas->pixmap() );

    fScaleFactor *= nFactor;
    fCanvas->resize( fScaleFactor * fCanvas->pixmap()->size() );

    AdjustScrollBar( fScrollArea->horizontalScrollBar(), nFactor );
    AdjustScrollBar( fScrollArea->verticalScrollBar(), nFactor );

    UpdateButtons();
}

// Handle zoom in click
void CVolumeViewer::OnZoomInClicked( void )
{
    ScaleImage( 1.25 );
}

// Handle zoom out click
void CVolumeViewer::OnZoomOutClicked( void )
{
    ScaleImage( 0.8 );
}

// Handle reset click
void CVolumeViewer::OnResetClicked( void )
{
    fCanvas->adjustSize();
    fScaleFactor = 1.0;

    UpdateButtons();
}

// Handle next image click
void CVolumeViewer::OnNextClicked( void )
{
    // emit signal to controller (MVC) in order to update the content
    emit SendSignalOnNextClicked();
}

// Handle previous image click
void CVolumeViewer::OnPrevClicked( void )
{
    // emit signal to controller (MVC ) in order to update the content
    emit SendSignalOnPrevClicked();
}

// Handle image index change
void CVolumeViewer::OnImageIndexEditTextChanged( void )
{
    // emit signal to controller in order to update the content
    emit SendSignalOnLoadAnyImage( fImageIndexEdit->GetImageIndex() );
}

// Update the status of the buttons
void CVolumeViewer::UpdateButtons( void )
{
    fZoomInBtn->setEnabled( fImgQImage != nullptr && fScaleFactor < 3.0 );
    fZoomOutBtn->setEnabled( fImgQImage != nullptr && fScaleFactor > 0.3333 );
    fResetBtn->setEnabled( fImgQImage != nullptr && fabs( fScaleFactor - 1.0 ) > 1e-6 );
    fNextBtn->setEnabled( fImgQImage != nullptr );
    fPrevBtn->setEnabled( fImgQImage != nullptr );
    //fImageIndexEdit->setEnabled( false );
    fImageIndexEdit->SetImageIndex( fImageIndex );
}

// Adjust scroll bar of scroll area
void CVolumeViewer::AdjustScrollBar( QScrollBar *nScrollBar,
                                   double nFactor )
{
    nScrollBar->setValue( int( nFactor * nScrollBar->value() + ( ( nFactor - 1 ) * nScrollBar->pageStep() / 2 ) ) );
}
