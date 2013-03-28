/**
 *
 * @param parent
 */
#include "centerwindow.h"
#include "ui_centerwindow.h"

/**
 *
 * @param parent
 */
CenterWindow::CenterWindow(QWidget *parent) :
    QMainWindow(parent), ui(new Ui::CenterWindow)
{
  ui->setupUi(this);
}

CenterWindow::~CenterWindow()
{
  delete ui;
}
