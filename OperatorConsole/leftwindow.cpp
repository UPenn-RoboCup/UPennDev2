#include "leftwindow.h"
#include "ui_leftwindow.h"

LeftWindow::LeftWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LeftWindow)
{
    ui->setupUi(this);
}

LeftWindow::~LeftWindow()
{
    delete ui;
}
