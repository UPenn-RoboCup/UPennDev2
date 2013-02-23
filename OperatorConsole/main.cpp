#include <QApplication>
#include <centerwindow.h>
#include <rightwindow.h>
#include <leftwindow.h>

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //TODO allocate to center screen
    CenterWindow centerWindow;
    centerWindow.show();

    //TODO allocate to rightmost screen
    RightWindow rightWindow;
    rightWindow.show();

    //TODO allocate to leftmost screen
    LeftWindow leftWindow;
    leftWindow.show();

    return a.exec();
}
