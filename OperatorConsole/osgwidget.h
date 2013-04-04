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

#include <QtCore/QTimer>
#include <osgViewer/Viewer>

/*!
 * \brief The OsgWidget class
 */

class OsgWidget : public QWidget, public osgViewer::Viewer
{
    Q_OBJECT
public:
    explicit OsgWidget();

    /*!
     * \brief Creates the OSG context and lays out the widget.
     */
    void initialize();

private:

signals:

public slots:
    /*!
     * \brief Requests OSG to render a frame
     */
    void drawFrame();

protected:
    /*!
     * \brief Timer for rendering frames (Will be removed!)
     */
    QTimer _timer;

private:
    /*!
     * \brief The camera used for this viewport
     */
    OsgCamera _camera;

};

#endif // OSGWIDGET_H
