/*!
 This QWindowClass represents the rightmost screen window.
 It has an associated UI file.

 initial class
 @author jdavis
 @date 02/21/2013

 TODO add layout
 */

#ifndef CENTERWINDOW_H
#define CENTERWINDOW_H

#include <QMainWindow>
#include <QGridLayout>
#include <QKeyEvent>

#include <phonon/MediaSource>
#include <phonon/VideoWidget>
#include <phonon/MediaObject>

namespace Ui {

class CenterWindow;
}

class CenterWindow : public QWidget
{
    Q_OBJECT

public:
    explicit CenterWindow(QWidget *parent = 0);
    ~CenterWindow();

    void keyPressEvent(QKeyEvent *key);

private:
    Phonon::VideoWidget *myVideoWidget;
    Phonon::MediaObject *myVideoPlayer;
};

#endif // CENTERWINDOW_H
