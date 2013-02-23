/*!
 This class represents the OpenSceneGraph Widget.

 It encapsulates an OpenSceneGraph view.

 @author jdavis
 @date 02/21/2013

 TODO build widget, use OsgWidgetQT example

 */

#ifndef OSGWIDGET_H
#define OSGWIDGET_H

#include <QWidget>
#include "osgcamera.h"

class OsgWidget : public QWidget
{
    Q_OBJECT
public:
    explicit OsgWidget(QWidget *parent = 0);
    
signals:
    
public slots:

private:
    OsgCamera _camera;

    
};

#endif // OSGWIDGET_H
