#include <QApplication>
#include <QDesktopWidget>
#include <centerwindow.h>
#include <keyboardeventfilter.h>
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
    XInitThreads();

    QApplication a(argc, argv);
    a.setApplicationName("Operator Console");

    // Get screen coordinates for each monitor
    QRect monitor1 = QApplication::desktop()->screenGeometry(0);
    QRect monitor2 = QApplication::desktop()->screenGeometry(1);
    QRect monitor3 = QApplication::desktop()->screenGeometry(2);

    // Create left window
    LeftWindow leftWindow;
    leftWindow.initialize();
    leftWindow.move(QPoint(monitor1.x(), monitor1.y()));
    leftWindow.window()->showFullScreen();
    leftWindow.show();

    // Create center window
    CenterWindow centerWindow;
    centerWindow.initialize();
    centerWindow.move(QPoint(monitor2.x(), monitor2.y()));
    centerWindow.window()->showFullScreen();
    centerWindow.show();

    // Create right window
    RightWindow rightWindow;
    rightWindow.initialize();
    rightWindow.move(QPoint(monitor3.x(), monitor3.y()));
    rightWindow.window()->showFullScreen();
    rightWindow.show();

    // Connect keyboard event filter to Application for exiting
    KeyboardEventFilter* kbdFilter = new KeyboardEventFilter();
    a.installEventFilter(kbdFilter);
    QObject::connect(kbdFilter, SIGNAL(escapePressed()), &a, SLOT(quit()));

    return a.exec();
}
