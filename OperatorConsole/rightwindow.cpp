#include "rightwindow.h"
#include "ui_rightwindow.h"

RightWindow::RightWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::RightWindow)
{
    ui->setupUi(this);
}

RightWindow::~RightWindow()
{
    delete ui;
}
