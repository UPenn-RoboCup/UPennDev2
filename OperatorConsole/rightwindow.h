/*!
 This QWindowClass represents the rightmost screen window.
 It has an associated UI file.

 @author jdavis
 @date 02/21/2013

 TODO add layout
 */

#ifndef RIGHTWINDOW_H
#define RIGHTWINDOW_H

#include <QMainWindow>

namespace Ui {
class RightWindow;
}

class RightWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit RightWindow(QWidget *parent = 0);
    ~RightWindow();
    
private:
    Ui::RightWindow *ui;
};

#endif // RIGHTWINDOW_H
