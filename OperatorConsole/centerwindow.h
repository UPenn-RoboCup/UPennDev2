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


namespace Ui {

class CenterWindow;
}

class CenterWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit CenterWindow(QWidget *parent = 0);
    ~CenterWindow();
    
private:
    Ui::CenterWindow *ui;
};

#endif // CENTERWINDOW_H
