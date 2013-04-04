/**
 *
 * @param parent
 */
#include "centerwindow.h"
#include "ui_centerwindow.h"

#include <QLabel>
#include <QGridLayout>

#include <phonon/MediaSource>

/**
 *
 * @param parent
 */
CenterWindow::CenterWindow(QWidget *parent) :
    QWidget(parent)
{

}

CenterWindow::~CenterWindow()
{

}

void CenterWindow::initialize()
{
    myVideoWidget = new Phonon::VideoWidget();
    myVideoPlayer = new Phonon::MediaObject(myVideoWidget);
    Phonon::createPath(myVideoPlayer, myVideoWidget);

    QLabel *labelLeft = new QLabel("Telemetry");
    labelLeft->setAlignment(Qt::AlignCenter);

    QLabel *labelRight = new QLabel("Telemetry");
    labelRight->setAlignment(Qt::AlignCenter);

    QGridLayout* grid = new QGridLayout;
    grid->addWidget( labelLeft, 0, 0 );
    grid->addWidget( myVideoWidget, 0, 1 );
    grid->addWidget( labelRight, 0, 2 );
    setLayout( grid );

    const Phonon::MediaSource *src = new Phonon::MediaSource("/home/afalendy/Videos/test.ogg");
    myVideoPlayer->setCurrentSource(*src);

    myVideoPlayer->play();
}
