#include <QApplication>
#include <QDesktopWidget>
#include <centerwindow.h>
#include <rightwindow.h>
#include <leftwindow.h>
#include <X11/Xlib.h>

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[])
{
//    XInitThreads();

//    osg::ArgumentParser arguments(&argc, argv);

//    osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::CullDrawThreadPerContext;
//    while (arguments.read("--SingleThreaded")) threadingModel = osgViewer::ViewerBase::SingleThreaded;
//    while (arguments.read("--CullDrawThreadPerContext")) threadingModel = osgViewer::ViewerBase::CullDrawThreadPerContext;
//    while (arguments.read("--DrawThreadPerContext")) threadingModel = osgViewer::ViewerBase::DrawThreadPerContext;
//    while (arguments.read("--CullThreadPerCameraDrawThreadPerContext")) threadingModel = osgViewer::ViewerBase::CullThreadPerCameraDrawThreadPerContext;

    QApplication a(argc, argv);
    a.setApplicationName("Operator Console");

    // Get screen coordinates for each monitor
    QRect monitor1 = QApplication::desktop()->screenGeometry(0);
    QRect monitor2 = QApplication::desktop()->screenGeometry(1);
    QRect monitor3 = QApplication::desktop()->screenGeometry(2);

    // Create center window
    CenterWindow centerWindow;
    centerWindow.move(QPoint(monitor2.x(), monitor2.y()));
    centerWindow.window()->showFullScreen();
    centerWindow.show();
    centerWindow.setFocus();

    // Create right window
    RightWindow rightWindow;
    rightWindow.move(QPoint(monitor3.x(), monitor3.y()));
    rightWindow.window()->showFullScreen();
    rightWindow.show();
    rightWindow.setFocus();

    // Create left window
    LeftWindow leftWindow;
    leftWindow.move(QPoint(monitor1.x(), monitor1.y()));
    leftWindow.window()->showFullScreen();
    leftWindow.show();
    leftWindow.setFocus();

    return a.exec();
}
